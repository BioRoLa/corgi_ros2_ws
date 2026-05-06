from setuptools import setup
from glob import glob
import os

package_name = "corgi_legwheel_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="anne",
    maintainer_email="r13522820@ntu.edu.tw",
    description="ROS 2 adapter package for LegWheel library integration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "legwheel_smoke = corgi_legwheel_ros.legwheel_smoke_node:main",
        ],
    },
)
