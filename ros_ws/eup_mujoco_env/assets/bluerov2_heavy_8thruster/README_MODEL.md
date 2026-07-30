# BlueROV2 Heavy 8-Thruster Model Notes

## Status

This directory reserves the target BlueROV2 Heavy-style 8-thruster model source.
The current repository uses `eup_mujoco_env/models/bluerov2_heavy_generic.xml`
as an equivalent simplified validation model.

## Source

TBD in Phase 0.

Candidate source types:

- Official or community URDF/Gazebo package.
- CAD or mesh export.
- Rebuilt MJCF from measured dimensions.

## License

TBD in Phase 0. Do not import third-party meshes until license and
redistribution rights are recorded here.

## Conversion Steps

Planned path:

1. Confirm source asset and license.
2. Normalize frames to ROS 2 `base_link`.
3. Convert or rebuild geometry in MJCF.
4. Add 8 actuator sites and verify numbering.
5. Add mass, inertia, buoyancy, and hydrodynamics assumptions.
6. Record known deviations from the physical vehicle.

## Thruster Layout

Initial placeholder numbering:

| ID | Name | Role |
| --- | --- | --- |
| 0 | front_left_vertical | vertical |
| 1 | front_right_vertical | vertical |
| 2 | rear_left_vertical | vertical |
| 3 | rear_right_vertical | vertical |
| 4 | front_left_horizontal | horizontal |
| 5 | front_right_horizontal | horizontal |
| 6 | rear_left_horizontal | horizontal |
| 7 | rear_right_horizontal | horizontal |

Signs, exact positions, and frame conventions must be confirmed against the
final model.

## Assumptions

- Mass and inertia are simplified.
- Hydrodynamics are not yet modeled beyond placeholder behavior.
- Buoyancy and drag must be added before dynamics validation.
- The simplified model is for interface and control-chain validation only.
