"""只读运行状态快照；不调整线程数、GC、浮点控制或算法参数。"""
import ctypes
import gc
import os
from pathlib import Path
import sys
import threading
import time
import tracemalloc

import numpy as np
from threadpoolctl import ThreadpoolController

_started = time.monotonic()
# NumPy/SciPy/Open3D在服务导入本模块前已加载；缓存库发现，info()仍实时查询线程数。
_pools = ThreadpoolController()
_gc_started = [0.0, 0.0, 0.0]
_gc_pause_seconds = [0.0, 0.0, 0.0]


def _track_gc(phase, info):
    generation = info["generation"]
    if phase == "start":
        _gc_started[generation] = time.perf_counter()
    else:
        _gc_pause_seconds[generation] += time.perf_counter() - _gc_started[generation]


def enable_gc_timing():
    """启动时登记GC耗时观测；不主动回收、不修改回收阈值。"""
    if _track_gc not in gc.callbacks:
        gc.callbacks.append(_track_gc)

if sys.platform == "win32":
    from ctypes import wintypes as W

    class _Memory(ctypes.Structure):
        _fields_ = [("cb", W.DWORD), ("pageFaults", W.DWORD)] + [
            (name, ctypes.c_size_t) for name in (
                "peakRss", "rss", "peakPagedPool", "pagedPool",
                "peakNonPagedPool", "nonPagedPool", "pagefile", "peakPagefile", "privateBytes")]

    class _ThreadEntry(ctypes.Structure):
        _fields_ = [("size", W.DWORD), ("usage", W.DWORD), ("id", W.DWORD),
                    ("owner", W.DWORD), ("base", W.LONG), ("delta", W.LONG), ("flags", W.DWORD)]

    class _Power(ctypes.Structure):
        _fields_ = [("acLine", W.BYTE), ("batteryFlags", W.BYTE), ("batteryPercent", W.BYTE),
                    ("batterySaver", W.BYTE), ("secondsLeft", W.DWORD), ("secondsFull", W.DWORD)]

    _kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    _psapi = ctypes.WinDLL("psapi", use_last_error=True)
    _psapi.GetProcessMemoryInfo.argtypes = [W.HANDLE, ctypes.POINTER(_Memory), W.DWORD]
    _kernel.GetProcessHandleCount.argtypes = [W.HANDLE, ctypes.POINTER(W.DWORD)]
    _kernel.CreateToolhelp32Snapshot.argtypes = [W.DWORD, W.DWORD]
    _kernel.CreateToolhelp32Snapshot.restype = W.HANDLE
    _kernel.Thread32First.argtypes = [W.HANDLE, ctypes.POINTER(_ThreadEntry)]
    _kernel.Thread32Next.argtypes = [W.HANDLE, ctypes.POINTER(_ThreadEntry)]
    _kernel.CloseHandle.argtypes = [W.HANDLE]
    _kernel.GetSystemPowerStatus.argtypes = [ctypes.POINTER(_Power)]
    _crt = ctypes.CDLL("ucrtbase")
    _crt._controlfp_s.argtypes = [ctypes.POINTER(ctypes.c_uint), ctypes.c_uint, ctypes.c_uint]


def snapshot(boundary="request-start"):
    """阶段边界读取自身状态；调用方隔离诊断失败，不读取业务数据或连接凭据。"""
    state = {
        "pid": os.getpid(), "nativeThreadId": threading.get_native_id(),
        "observerAgeSeconds": round(time.monotonic() - _started, 3),
        "pythonThreads": threading.active_count(), "gcEnabled": gc.isenabled(),
        "gcCounts": gc.get_count(), "gcStats": gc.get_stats(),
        "gcPauseMs": [round(seconds * 1000, 3) for seconds in _gc_pause_seconds],
        "gcTimingEnabled": _track_gc in gc.callbacks, "tracemallocEnabled": tracemalloc.is_tracing(),
        "switchIntervalSeconds": sys.getswitchinterval(), "numpyErrorMode": np.geterr(),
        "traceEnabled": sys.gettrace() is not None, "profileEnabled": sys.getprofile() is not None,
        "numericPools": _pools.info(),
    }
    if sys.platform == "win32":
        memory = _Memory()
        memory.cb = ctypes.sizeof(memory)
        if not _psapi.GetProcessMemoryInfo(-1, ctypes.byref(memory), memory.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        handles = W.DWORD()
        if not _kernel.GetProcessHandleCount(-1, ctypes.byref(handles)):
            raise ctypes.WinError(ctypes.get_last_error())
        control = ctypes.c_uint()
        status = _crt._controlfp_s(ctypes.byref(control), 0, 0)  # mask=0，仅查询当前线程。
        state.update(rssBytes=memory.rss, peakRssBytes=memory.peakRss,
                     privateBytes=memory.privateBytes, pageFaults=memory.pageFaults,
                     handles=handles.value, floatControl=control.value, floatControlStatus=status)
        power = _Power()
        if not _kernel.GetSystemPowerStatus(ctypes.byref(power)):
            raise ctypes.WinError(ctypes.get_last_error())
        state["power"] = {"acLine": power.acLine, "batteryPercent": power.batteryPercent,
                          "batterySaver": power.batterySaver}
        # Windows线程枚举涉及全机线程，仅在请求首尾执行，避免给每个算法阶段增加开销。
        if boundary in ("request-start", "request-end"):
            handle = _kernel.CreateToolhelp32Snapshot(4, 0)
            if handle == ctypes.c_void_p(-1).value:
                raise ctypes.WinError(ctypes.get_last_error())
            entry = _ThreadEntry()
            entry.size = ctypes.sizeof(entry)
            count = 0
            try:
                valid = _kernel.Thread32First(handle, ctypes.byref(entry))
                while valid:
                    count += entry.owner == os.getpid()
                    valid = _kernel.Thread32Next(handle, ctypes.byref(entry))
            finally:
                _kernel.CloseHandle(handle)
            state["nativeThreads"] = count
    elif sys.platform.startswith("linux"):
        import resource
        usage = resource.getrusage(resource.RUSAGE_SELF)
        status = dict(line.split(":", 1) for line in Path("/proc/self/status").read_text().splitlines())
        state.update(rssBytes=int(status["VmRSS"].split()[0]) * 1024,
                     peakRssBytes=int(status["VmHWM"].split()[0]) * 1024,
                     nativeThreads=int(status["Threads"]), minorFaults=usage.ru_minflt,
                     majorFaults=usage.ru_majflt, voluntarySwitches=usage.ru_nvcsw,
                     involuntarySwitches=usage.ru_nivcsw)
    return state
