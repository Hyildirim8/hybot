#!/usr/bin/env python3
"""Add the CSI camera dock to a temporary PC copy of the shared RViz config."""
import argparse
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error("output must differ from the shared source config")

    config = yaml.safe_load(args.source.read_text())
    displays = config["Visualization Manager"]["Displays"]
    displays[:] = [display for display in displays if display.get("Name") != "RPi Camera"]
    displays.append({
        "Class": "rviz_default_plugins/Image",
        "Name": "RPi Camera",
        "Enabled": True,
        "Value": True,
        "Normalize Range": True,
        "Min Value": 0,
        "Max Value": 1,
        "Median window": 5,
        # Humble derives image_transport's plugin from the topic suffix.
        # Match the camera publisher's BEST_EFFORT QoS and keep latency low.
        "Topic": {
            "Value": "/camera_csi/image_raw/compressed",
            "Reliability Policy": "Best Effort",
            "Durability Policy": "Volatile",
            "History Policy": "Keep Last",
            "Depth": 2,
        },
    })
    args.output.write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == "__main__":
    main()
