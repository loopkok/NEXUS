"""The one-shot NEXUS start must reach the mux and every arm adapter."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from astral_web_monitor.monitor_node import MonitorNode  # noqa: E402


class _Publisher:
    def __init__(self, counts=()):
        self.counts = iter(counts)
        self.last_count = 0
        self.messages = []

    def get_subscription_count(self):
        self.last_count = next(self.counts, self.last_count)
        return self.last_count

    def publish(self, msg):
        self.messages.append(msg.data)


def _node(start):
    node = object.__new__(MonitorNode)
    node._nexus_publishers = {"teleop_armed": _Publisher(), "teleop_start": start}
    node._nexus_start_subscribers = 3  # mux + left IK + right IK
    return node


def test_start_waits_for_both_arm_adapters():
    start = _Publisher([1, 2, 3])
    node = _node(start)
    assert node.publish_nexus_teleop_start(timeout_s=0.2)
    assert start.messages == [True]


def test_start_fails_when_an_arm_adapter_is_missing():
    start = _Publisher([2])
    node = _node(start)
    assert not node.publish_nexus_teleop_start(timeout_s=0)
    assert start.messages == []
