"""Nero homing configuration, independent of ROS and the hardware SDK."""

import math


def validate_config(config):
    """Profile validation hook; Nero arms always have seven joints."""
    value = config.get("home_tolerance_rad", 0.05)
    if isinstance(value, dict) and not value:
        raise ValueError("home_tolerance_rad: side mapping must not be empty")
    sides = value if isinstance(value, dict) else ("left",)
    for side in sides:
        home_tolerances(config, side, 7)


def home_tolerances(config, side, dimension):
    """Resolve a scalar, joint list or per-side mapping to radians per joint."""
    field = "adapter_config.nero_can.home_tolerance_rad"
    value = config.get("home_tolerance_rad", 0.05)
    if isinstance(value, dict):
        if set(value) - {"left", "right"}:
            raise ValueError(f"{field}: only left/right side keys are allowed")
        if side not in value:
            raise ValueError(f"{field}: missing {side} arm tolerance")
        value = value[side]
        field += f".{side}"
    values = value if isinstance(value, list) else [value] * dimension
    if len(values) != dimension:
        raise ValueError(f"{field}: expected {dimension} values in component joint order")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError(f"{field}: every tolerance must be a finite positive number in radians")
    return tuple(float(v) for v in values)
