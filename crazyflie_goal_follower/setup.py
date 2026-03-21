import os
from glob import glob

from setuptools import find_packages, setup

package_name = "crazyflie_goal_follower"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="argonaut",
    maintainer_email="argonaut@todo.todo",
    description="Goal follower node for Crazyflie using go_to/takeoff services.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goal_follower_node = crazyflie_goal_follower.goal_follower_node:main",
        ],
    },
)
