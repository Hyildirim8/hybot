from setuptools import setup

package_name = "ecza_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ecza-robotu",
    description="Joy → cmd_vel teleop node for the ecza mecanum rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"teleop_node = {package_name}.teleop_node:main",
            f"slam_manager_node = {package_name}.slam_manager_node:main",
        ],
    },
)
