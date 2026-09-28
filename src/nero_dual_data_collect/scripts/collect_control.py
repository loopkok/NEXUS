#!/usr/bin/env python3
"""Keyboard controller via TCP socket. Zero dependency on ROS2."""

import sys
import socket
import tty
import termios

HOST, PORT = "127.0.0.1", 19999


def get_char():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def send_cmd(cmd, label, color):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.connect((HOST, PORT))
        s.sendall(cmd.encode())
        s.close()
        print(f"\033[{color}m[{label}]\033[0m")
    except Exception as e:
        print(f"\033[31m[ERR: {e}]\033[0m")


def main():
    print("\033[1;36m" + "=" * 40 + "\033[0m")
    print("\033[1;32m  Data Collection Keyboard Control\033[0m")
    print("  \033[33ms\033[0m  — Start / Resume")
    print("  \033[33mp\033[0m  — Pause")
    print("  \033[34mq\033[0m  — Stop & Save")
    print("  \033[31md\033[0m  — Discard")
    print("  \033[36mn\033[0m  — Next episode")
    print("  \033[37mESC\033[0m — Quit")
    print("\033[1;36m" + "=" * 40 + "\033[0m")

    key_map = {
        's': ("start", "START", 32),
        'p': ("pause", "PAUSE", 33),
        'q': ("stop", "STOP & SAVE", 34),
        'd': ("discard", "DISCARD", 31),
        'n': ("next", "NEXT EPISODE", 36),
    }

    try:
        while True:
            ch = get_char().lower()
            if ch in ('\x1b', '\x03'):
                print("\nQuit.")
                break
            elif ch in key_map:
                cmd, label, color = key_map[ch]
                send_cmd(cmd, label, color)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
