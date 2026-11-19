"""拆分后保留旧入口，底层模块导入不触发设备、网络或数据库操作。"""
from pathlib import Path
import subprocess
import sys
import unittest


class TestArchitectureBoundaries(unittest.TestCase):
    def test_imports_do_not_start_io_and_lower_layers_stay_independent(self):
        script = '''
import socket, sqlite3, threading, sys
def unexpected(*args, **kwargs):
    raise AssertionError("import started I/O")
class UnexpectedSocket(socket.socket):
    __new__ = staticmethod(unexpected)
socket.socket = UnexpectedSocket
sqlite3.connect = threading.Thread.start = unexpected
from app.application.events import EventBus
from app.application.channel_state import ChannelState
assert "numpy" not in sys.modules
assert "app.service" not in sys.modules
from app.adapters.sqlite_outbox import SQLiteOutbox
assert "app.adapters.webapi_http" not in sys.modules
from app import load_radar_transport, service
assert load_radar_transport.SQLiteOutbox is SQLiteOutbox
assert service.EventBus is EventBus
assert service.ChannelState is ChannelState
'''
        result = subprocess.run([sys.executable, '-c', script],
            cwd=Path(__file__).resolve().parents[1], capture_output=True, timeout=30)
        self.assertEqual(0, result.returncode, result.stderr.decode('utf-8', errors='replace'))

    def test_existing_service_entry_points_are_the_same_objects(self):
        from app import service
        from app.application.channel_worker import ChannelWorker
        from app.adapters.console_logger import ConsoleLogger, attach_console_logger

        self.assertIs(service.ChannelWorker, ChannelWorker)
        self.assertIs(service.ConsoleLogger, ConsoleLogger)
        self.assertIs(service.attach_console_logger, attach_console_logger)
