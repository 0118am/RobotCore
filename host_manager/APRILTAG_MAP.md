# Managed AprilTag map

The host manager owns one JSON document at the root-configured
`apriltag_map.path`. AprilTag localisation accepts JSON map files only; this
managed document is the single map authority. Each tag is an upsertable map
definition:

```json
{
  "schema_version": 1,
  "frame": "map",
  "tags": {
    "12": {
      "position_m": [1.2, -0.4, 0.3],
      "rpy_deg": [90.0, 0.0, -90.0],
      "size_m": 0.13
    }
  }
}
```

`position_m` is the **centre of the black outer square** in the `map` frame.
`rpy_deg` describes the ROS tag frame: printed right is +X, printed top is +Y,
and the face normal is +Z. `size_m` is the measured black-square edge length,
not the paper width.

`rpy_deg` always describes the physical printed axes. Do not add a 180-degree
rotation to compensate for OpenCV's internal `DICT_APRILTAG_*` corner
convention. The localization detector normalizes that convention before PnP.

The deployed pool geometry is `F=[0, 5.42] m`, `L=[0, 3.73] m`, with the
floor at `U=0`. Localization validates every complete map revision before
adopting it: each Tag centre must lie on the floor or one of the four inner
walls, its face normal must point into the pool, wall-tag printed top must
point toward `+U`, and its four black-square corners must remain in bounds.

The web panel reads the managed file once on load and after explicit map
actions. It can add or update one ID when
`apriltag_map.web_edit_enabled` is true (the robot operator configuration
enables this by default). The daemon validates ranges, writes a temporary
file, fsyncs it, and atomically replaces the managed map.
`apriltag_localization_node` loads the map once at startup. Saved edits take
effect only after the explicit Relocalize action reloads and validates the
complete map; a failed reload preserves the last valid map.

For a local maintenance entry, use:

```bash
robotcore-hostctl apriltag-upsert --tag-id 12 --size-m 0.13 \
  --position-m 1.2 -0.4 0.3 --rpy-deg 90 0 -90
```

To remove a definition, use the same managed path:

```bash
robotcore-hostctl apriltag-delete --tag-id 12
```

Deletion atomically writes the remaining map. An empty, valid map is also
reloaded, so deleting the final tag removes it from
localisation rather than retaining a stale layout.

An empty map is a deliberate fail-closed state: the localizer clears the prior
layout, resets map alignment, continues publishing detection counts/status, and
publishes no absolute pose. A missing, malformed, wrong-frame, or geometrically
invalid replacement is rejected and cannot overwrite the last valid in-memory
revision.

The repository does not provide a fallback map or synthesize coordinates. Each
non-empty deployed revision must contain `schema_version: 1`, `frame: "map"`, a
canonical unsigned ID, and an explicit measured `size_m` for every Tag. With
cuboid validation enabled, every centre and black-square corner must lie on or
inside the configured pool surfaces; face normals must point into the pool and
wall-mounted printed tops must point upward.

Before a map revision is approved for automatic control:

1. Survey every black-square centre, printed-axis orientation, and edge length
   from the same physical datum; record the tool, operator, date, and map hash.
2. Run the on-robot structural/geometry check:

   ```bash
   ROBOTCORE_TEST_DEPLOYED_TAG_MAP=/etc/robotcore/apriltag_map.json \
     /home/nvidia/RobotCore/ros_ws/build/robotcore_sensors/test_apriltag_map \
     --gtest_filter=AprilTagMap.DeployedSurveyCanBeValidatedExplicitly
   ```

3. With independent ground truth, meet the localization acceptance limits:
   position RMSE at most `0.10 m`, attitude RMSE at most `3 deg`, and p95 at
   most `0.15 m / 5 deg`; verify at least three mapped Tags across every
   intended operating region. A parser pass alone is not evidence of survey
   accuracy.
