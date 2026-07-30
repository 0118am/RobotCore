"""Action decoding helpers for policy outputs.

Runners return plain numeric vectors. These helpers clamp and shape those
vectors before control nodes publish concrete ROS command messages.
"""

def clamp_normalized(values, size):
    # Clamp each model output defensively; a real policy can fail open during
    # early integration, but control topics should stay in the declared range.
    output = [0.0] * size
    for index, value in enumerate(list(values)[:size]):
        output[index] = max(-1.0, min(1.0, float(value)))
    return output


def decode_thruster_action(action):
    return clamp_normalized(action, 8)


def decode_arm_joint_targets(action, joint_count):
    # Missing joints default to zero so dummy and partially implemented runners
    # can still exercise the arm command path.
    output = [0.0] * joint_count
    for index, value in enumerate(list(action)[:joint_count]):
        output[index] = float(value)
    return output
