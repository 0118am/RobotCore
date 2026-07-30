#!/usr/bin/env python3
"""Create a run directory that matches the acceptance logging contract."""

import json
from datetime import datetime
from pathlib import Path


def create_run_dir(root: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = root / f"run_{stamp}"
    # Keep this list synchronized with ACCEPTANCE.md and RunLogger.
    for subdir in ["rosbag2", "policy_io", "captures", "configs"]:
        (run_dir / subdir).mkdir(parents=True, exist_ok=True)
    (run_dir / "event_log.jsonl").touch()
    snapshot = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "layout_version": "0.1.0",
        "required_subdirs": ["rosbag2", "policy_io", "captures", "configs"],
    }
    (run_dir / "configs" / "run_snapshot.json").write_text(
        json.dumps(snapshot, indent=2) + "\n", encoding="utf-8"
    )
    return run_dir


def main():
    # The script is intentionally workspace-relative for developer smoke tests.
    root = Path("data/robotcore_runs")
    run_dir = create_run_dir(root)
    print(run_dir)


if __name__ == "__main__":
    main()
