import os
from glob import glob

from setuptools import setup

package_name = "rosorin_nav2"


def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", files("launch/*.py")),
        ("share/" + package_name + "/config", files("config/*.yaml")),
        ("share/" + package_name + "/maps", files("maps/*.yaml") + files("maps/*.pgm")),
        ("share/" + package_name + "/rviz", files("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matin Esmaeili",
    maintainer_email="mn.matinesmaeili@gmail.com",
    description="Nav2 configuration for the ROSOrin Pro",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "goal = rosorin_nav2.goal:main",
        ],
    },
)
