"""Pure helpers for simulator lifecycle command ordering."""

from __future__ import annotations


def is_pre_home_command(msg, home_stamp_ns: int) -> bool:
    """Whether a stamped JointState command predates the active home request."""
    stamp = msg.header.stamp
    command_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return command_stamp_ns <= home_stamp_ns
