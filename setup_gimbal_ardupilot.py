#!/usr/bin/env python3
"""Modify gimbal_small_3d camera sensor for the ArduPilot simulation.

The iris_with_gimbal model (from ardupilot_gazebo) already includes a 3-axis
gimbal with camera. This script tweaks the camera sensor:
  1. Adds an explicit <topic>/drone/camera</topic> for a known Gazebo topic name
  2. Removes CameraZoomPlugin and GstCameraPlugin (not built in our Docker)
  3. Increases update_rate from 10 to 30 Hz for smoother video
"""

import xml.etree.ElementTree as ET

GIMBAL_MODEL_PATH = "/root/ardupilot_gazebo/models/gimbal_small_3d/model.sdf"


def main():
    tree = ET.parse(GIMBAL_MODEL_PATH)
    root = tree.getroot()
    model = root.find("model")

    # Find the camera sensor on pitch_link
    for link in model.findall("link"):
        if link.get("name") != "pitch_link":
            continue
        for sensor in link.findall("sensor"):
            if sensor.get("name") == "camera" and sensor.get("type") == "camera":
                # Add explicit topic so we know the Gazebo transport topic name
                topic = sensor.find("topic")
                if topic is None:
                    topic = ET.SubElement(sensor, "topic")
                topic.text = "/drone/camera"

                # Increase update rate from 10 to 30 Hz
                rate = sensor.find("update_rate")
                if rate is not None:
                    rate.text = "30"

                # Remove plugins that aren't built in our Docker
                for plugin in list(sensor.findall("plugin")):
                    name = plugin.get("filename", "") + plugin.get("name", "")
                    if "CameraZoom" in name or "GstCamera" in name:
                        sensor.remove(plugin)
                        print(f"  Removed plugin: {plugin.get('filename')}")

                print(f"  topic: /drone/camera")
                print(f"  update_rate: 30 Hz")

    ET.indent(tree, space="  ")
    tree.write(GIMBAL_MODEL_PATH, xml_declaration=True, encoding="UTF-8")
    print(f"Gimbal camera configured in {GIMBAL_MODEL_PATH}")


if __name__ == "__main__":
    main()
