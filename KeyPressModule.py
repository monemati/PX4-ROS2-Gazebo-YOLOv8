import os
import sys
import tty
import termios
import select
import threading
import time

_active_keys = {}
_KEY_TIMEOUT = 0.2
_lock = threading.Lock()
_running = True
_fd = None
_old_settings = None

_ESCAPE_MAP = {
    b'A': 'UP',
    b'B': 'DOWN',
    b'C': 'RIGHT',
    b'D': 'LEFT',
}


def _reader_thread():
    global _running
    try:
        while _running:
            if select.select([_fd], [], [], 0.05)[0]:
                ch = os.read(_fd, 1)
                if ch == b'\x1b':
                    if select.select([_fd], [], [], 0.05)[0]:
                        ch2 = os.read(_fd, 1)
                        if ch2 == b'[':
                            if select.select([_fd], [], [], 0.05)[0]:
                                ch3 = os.read(_fd, 1)
                                key_name = _ESCAPE_MAP.get(ch3)
                                if key_name:
                                    with _lock:
                                        _active_keys[key_name] = time.time()
                elif ch == b'\x03':
                    _running = False
                    _restore()
                    os._exit(0)
                else:
                    with _lock:
                        _active_keys[ch.decode('ascii', errors='ignore').lower()] = time.time()
    except Exception:
        pass


def _restore():
    if _old_settings is not None and _fd is not None:
        try:
            termios.tcsetattr(_fd, termios.TCSADRAIN, _old_settings)
        except Exception:
            pass


def init():
    global _fd, _old_settings
    _fd = sys.stdin.fileno()
    _old_settings = termios.tcgetattr(_fd)
    tty.setcbreak(_fd)

    t = threading.Thread(target=_reader_thread, daemon=True)
    t.start()

    sys.stdout.write("\n=== Drone Keyboard Control ===\n")
    sys.stdout.write("  r        : Arm\n")
    sys.stdout.write("  l        : Land\n")
    sys.stdout.write("  w / s    : Throttle up / down\n")
    sys.stdout.write("  a / d    : Yaw left / right\n")
    sys.stdout.write("  Arrows   : Roll / Pitch\n")
    sys.stdout.write("  j / k    : Gimbal pitch down / up\n")
    sys.stdout.write("  n / m    : Gimbal yaw left / right\n")
    sys.stdout.write("  i        : Print flight mode\n")
    sys.stdout.write("  1        : Loiter mode\n")
    sys.stdout.write("  2        : AltHold mode\n")
    sys.stdout.write("  3        : Guided mode\n")
    sys.stdout.write("  4        : Stabilize mode\n")
    sys.stdout.write("  5        : Land mode\n")
    sys.stdout.write("  Ctrl+C   : Quit\n")
    sys.stdout.write("\n")
    sys.stdout.write("  Tmux: Ctrl+b then arrow keys to switch panes\n")
    sys.stdout.write("==============================\n\n")
    sys.stdout.flush()


def getKey(keyName):
    with _lock:
        ts = _active_keys.get(keyName)
        if ts is None:
            return False
        if time.time() - ts > _KEY_TIMEOUT:
            del _active_keys[keyName]
            return False
        return True


if __name__ == '__main__':
    init()
    sys.stdout.write("Testing keyboard input. Press keys (Ctrl+C to quit)...\n")
    sys.stdout.flush()
    try:
        while _running:
            for key in ['UP', 'DOWN', 'LEFT', 'RIGHT', 'w', 'a', 's', 'd', 'r', 'l']:
                if getKey(key):
                    sys.stdout.write(f"  {key} pressed\r\n")
                    sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        _restore()
