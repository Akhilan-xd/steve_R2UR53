from setuptools import setup

package_name = "steve_simulation"

setup(
    name=package_name,
    version="1.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "launch/simulation.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Akhilan Ashokan",
    maintainer_email="akhilan.ashokan@smail.inf.h-brs.de",
    description="Gazebo Classic simulation for the Steve mobile manipulator (Neobotix MMO-700 with UR5e)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
