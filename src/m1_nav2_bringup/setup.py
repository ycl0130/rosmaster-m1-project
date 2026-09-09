from glob import glob
import os

from setuptools import setup


package_name = "m1_nav2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Rosmaster user",
    maintainer_email="user@example.com",
    description="Nav2 and SLAM bringup for the Rosmaster M1 Gazebo baseline.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "initial_pose_publisher = m1_nav2_bringup.initial_pose_publisher:main",
            "scan_relay = m1_nav2_bringup.scan_relay:main",
        ],
    },
)
