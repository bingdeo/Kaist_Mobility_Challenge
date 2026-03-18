from setuptools import setup
import os
from glob import glob

package_name = "pkg_p1_1"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/waypoints", glob("waypoints/*.json")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smyd",
    maintainer_email="inhsroy@hanyang.ac.kr",
    description="Problem 1-1 controller",
    license="TODO",
    entry_points={
        "console_scripts": [
            "p1_1 = pkg_p1_1.p1_1:main",
        ],
    },
)
