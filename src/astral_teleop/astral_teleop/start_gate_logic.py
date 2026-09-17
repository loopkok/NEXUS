"""Pure decision: left Touch gripClick → teleop start vs HITL takeover.

When the policy node is *active* (``/policy_inference/state`` activity ∈
{policy, playback}), the left grip must NOT arm teleop — ``/teleop/start``
would re-arm the arm teleop node and race the policy writing
``joint_commands`` (last-writer-wins, 150Hz teleop vs 30Hz policy). Instead
grip becomes the HITL takeover trigger (``/policy_inference/cmd`` =
"takeover"). Otherwise it keeps starting teleop (re-capture vr_init + arm).

No ROS here — pure decision, offline-testable.
"""

from __future__ import annotations

from typing import Optional

# Policy/playback running: the arm is owned by the policy — grip = takeover.
ACTIVE_ACTIVITIES = frozenset(("policy", "playback"))


def decide_start_action(activity: Optional[str]) -> str:
    """Return ``"takeover"`` when policy/playback is active, else
    ``"teleop_start"``.

    ``activity`` comes from ``/policy_inference/state`` (None when the policy
    node is not running / no state yet).
    """
    if activity in ACTIVE_ACTIVITIES:
        return "takeover"
    return "teleop_start"
