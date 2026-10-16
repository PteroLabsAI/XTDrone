from glob import glob

from setuptools import setup

PACKAGE = "pterosim_ego"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE}/config", glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    # colcon test runs pytest only for a package that names it; setuptools 72+ drops tests_require.
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    entry_points={
        "console_scripts": [
            f"depth_bridge = {PACKAGE}.depth_bridge:main",
            f"px4_bridge = {PACKAGE}.px4_bridge:main",
            f"start_swarm = {PACKAGE}.start_swarm:main",
            f"pterosim_clock = {PACKAGE}.pterosim_clock:main",
        ],
    },
)
