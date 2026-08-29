"""Strict validation for direct policy actions."""

import math


def decode_thruster_action(action):
    values = [float(value) for value in action]
    if len(values) != 8:
        raise ValueError("policy action must contain exactly eight values")
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in values):
        raise ValueError("policy action must contain finite values in [-1, 1]")
    return values
