"""Linux IPv4 UDP 批量接收。每个报文沿用内核入队时间，排队期间不刷新时间戳。"""
import ctypes as ct
import errno
import os
import platform
import select
import socket
import struct
import sys
import time


class _IOVec(ct.Structure):
    _fields_ = [("base", ct.c_void_p), ("size", ct.c_size_t)]


class _MsgHdr(ct.Structure):
    _fields_ = [("name", ct.c_void_p), ("namelen", ct.c_uint),
                ("iov", ct.c_void_p), ("iovlen", ct.c_size_t),
                ("control", ct.c_void_p), ("controllen", ct.c_size_t), ("flags", ct.c_int)]


class _MMsgHdr(ct.Structure):
    _fields_ = [("msg", _MsgHdr), ("length", ct.c_uint)]


class LinuxUdpBatch:
    """单读者独占一个套接字。Linux LP64 的 msghdr、sockaddr_in 和 timespec 布局。"""
    capacity = 128
    timestamping = 37  # Linux LP64 SO_TIMESTAMPING，三个 timespec 各有两个 64 位 long。
    rx_software = (1 << 3) | (1 << 4)  # SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE

    @classmethod
    def for_socket(cls, sock, payload_size):
        # 其他 ABI 继续使用 Python socket，避免用错结构体或时间戳选项。
        if (not sys.platform.startswith("linux") or platform.machine() not in ("x86_64", "aarch64")
                or ct.sizeof(ct.c_long) != 8):
            return None
        try:
            receive = ct.CDLL(None, use_errno=True).recvmmsg
        except AttributeError:
            return None
        try:
            sock.setsockopt(socket.SOL_SOCKET, cls.timestamping, cls.rx_software)
        except OSError:
            return None
        # Linux 首次开启接收时间戳时异步激活内核网络路径。正式端口 bind 前，
        # 在独立回环端口确认能拿到真实入队时刻；测试报文不会进现场队列。
        # 单独使用 SO_TIMESTAMPING，避免 SO_TIMESTAMPNS 在缺时标时以取包时间补造。
        try:
            cls._prime_kernel_timestamps()
        except OSError:
            # 回环探测受限时仍可收真实设备报文，缺时标将逐包记数。
            pass
        return cls(sock, payload_size, receive)

    @classmethod
    def _prime_kernel_timestamps(cls):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, cls.timestamping, cls.rx_software)
            probe.bind(("127.0.0.1", 0))
            probe.settimeout(0.02)
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                probe.sendto(b"x", probe.getsockname())
                try:
                    _, control, _, _ = probe.recvmsg(1, 64)
                except socket.timeout:
                    continue
                if any(level == socket.SOL_SOCKET and kind == cls.timestamping and len(value) == 48
                       and any(value[:16]) for level, kind, value in control):
                    return
                time.sleep(0.002)

    def __init__(self, sock, payload_size, receive):
        self.sock = sock
        self._fd = sock.fileno()
        self._poll = select.poll()
        self._poll.register(self._fd, select.POLLIN)
        receive.argtypes = [ct.c_int, ct.POINTER(_MMsgHdr), ct.c_uint, ct.c_int, ct.c_void_p]
        receive.restype = ct.c_int
        self._receive = receive
        self._payload_size = payload_size
        self._timestamp = struct.Struct("@Niill")  # cmsghdr + 第一个 timespec（软件 RX）
        self._control_size = socket.CMSG_SPACE(3 * struct.calcsize("@ll"))
        control_size = self._control_size
        self._packets = (ct.c_ubyte * (payload_size * self.capacity))()
        self._addresses = (ct.c_ubyte * (16 * self.capacity))()
        self._controls = (ct.c_ubyte * (control_size * self.capacity))()
        self._vec = (_IOVec * self.capacity)()
        self._msgs = (_MMsgHdr * self.capacity)()
        for i, slot in enumerate(self._msgs):
            self._vec[i] = _IOVec(ct.addressof(self._packets) + i * payload_size, payload_size)
            slot.msg = _MsgHdr(ct.addressof(self._addresses) + i * 16, 16,
                               ct.addressof(self._vec) + i * ct.sizeof(_IOVec), 1,
                               ct.addressof(self._controls) + i * control_size, control_size, 0)
        self._data = memoryview(self._packets).cast("B")
        self._ips = memoryview(self._addresses).cast("B")
        self.invalid_timestamps = 0

    def read(self):
        # socket.settimeout 将 fd 置为非阻塞；先 poll 避免无数据时空转或停机时无限等待。
        if not self._poll.poll(50):
            return []
        n = self._receive(self._fd, self._msgs, self.capacity, socket.MSG_DONTWAIT, None)
        if n < 0:
            err = ct.get_errno()
            if err in (errno.EINTR, errno.EAGAIN, errno.EWOULDBLOCK):
                return []
            raise OSError(err, os.strerror(err))
        items = []
        control_size = self._control_size
        for i in range(n):
            slot = self._msgs[i]
            cmsg_len, level, kind, seconds, ns = self._timestamp.unpack_from(self._controls, i * control_size)
            valid = (slot.msg.controllen == control_size and cmsg_len == control_size
                     and level == socket.SOL_SOCKET and kind == self.timestamping
                     and seconds > 0 and 0 <= ns < 1_000_000_000 and not slot.msg.flags & socket.MSG_CTRUNC)
            slot.msg.controllen = control_size
            slot.msg.namelen = 16
            if not valid:
                # 缺少可信时标的包不能被当成刚到达的停稳证据。
                self.invalid_timestamps += 1
                continue
            src = socket.inet_ntoa(self._ips[i * 16 + 4:i * 16 + 8])
            start = i * self._payload_size
            data = self._data[start:start + slot.length].tobytes()
            items.append((src, data, seconds + ns / 1_000_000_000))
        return items
