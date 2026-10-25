from glob import glob
from setuptools import setup

package_name = "diffdrive_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zahrah Fauziah",
    description="Differential drive kinematics simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wheel_commander = diffdrive_sim.wheel_commander:main",
            "kinematics_node = diffdrive_sim.kinematics_node:main",
            "slider_gui = diffdrive_sim.slider_gui:main",
            "goal_gui = diffdrive_sim.goal_gui:main",
        ],
    },
)
