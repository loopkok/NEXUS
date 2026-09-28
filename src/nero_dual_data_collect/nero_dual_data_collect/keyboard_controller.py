"""Keyboard input controller for data collection control.

Provides non-blocking keyboard input in a background thread.
Supports raw terminal mode on Linux and msvcrt on Windows.
"""

import sys
import threading
import time
from typing import Optional


class KeyboardController:
    """Thread-based keyboard input handler for data collection control.

    Key bindings:
        s - Start recording
        p - Pause/Resume recording
        q - Stop and save episode
        d - Discard current episode
        n - Next episode (stop current, start new)
    """

    def __init__(self):
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._last_key: Optional[str] = None
        self._lock = threading.Lock()

    @staticmethod
    def _get_char() -> str:
        """Read a single character from stdin (cross-platform)."""
        try:
            import msvcrt
            return msvcrt.getch().decode("utf-8").lower()
        except ImportError:
            import tty
            import termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1).lower()
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    def _run(self):
        """Main loop: continuously read keyboard input."""
        while self._running:
            try:
                ch = self._get_char()
                with self._lock:
                    self._last_key = ch
                # Ctrl+C handling
                if ch == "\x03":
                    self._running = False
            except (IOError, OSError):
                time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

    def start(self):
        """Start the keyboard listening thread."""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the keyboard listening thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def get_key(self) -> Optional[str]:
        """Get and clear the last pressed key (non-blocking)."""
        with self._lock:
            key = self._last_key
            self._last_key = None
            return key

    def wait_for_key(self, timeout: float = None) -> Optional[str]:
        """Block until a key is pressed, or timeout expires."""
        start = time.time()
        while self._running:
            key = self.get_key()
            if key is not None:
                return key
            if timeout is not None and (time.time() - start) > timeout:
                return None
            time.sleep(0.05)
        return None


def print_colored(text: str, color: int = 37, bold: bool = False):
    """Print colored text to terminal using ANSI escape codes.

    Colors: 31=red, 32=green, 33=yellow, 34=blue, 35=magenta, 36=cyan, 37=white
    """
    fmt = 1 if bold else 0
    print(f"\033[{fmt};{color}m{text}\033[0m")


def print_status(state: str):
    """Print data collection status with appropriate color."""
    colors = {
        "idle": 37,       # white
        "ready": 36,      # cyan
        "recording": 32,  # green
        "paused": 33,     # yellow
        "saving": 34,     # blue
        "discard": 31,    # red
        "error": 31,      # red
    }
    color = colors.get(state, 37)
    print_colored(f"[{state.upper()}]", color, bold=True)


def print_help():
    """Print key binding help."""
    print_colored("=" * 50, 36)
    print_colored("  Data Collection Controls:", 36, bold=True)
    print_colored("    [s] Start recording", 32)
    print_colored("    [p] Pause / Resume", 33)
    print_colored("    [q] Stop & Save episode", 34)
    print_colored("    [d] Discard episode & retry", 31)
    print_colored("    [n] Stop current, start new episode", 36)
    print_colored("    [Ctrl+C] Quit", 37)
    print_colored("=" * 50, 36)
