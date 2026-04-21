from setuptools import setup

package_name = "ecza_kinematics"

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
    description="Mecanum wheel inverse kinematics for ecza rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"kinematics_node = {package_name}.kinematics_node:main",
            f"mecanum_translator_node = {package_name}.mecanum_translator_node:main",
            f"encoder_verifier_node = {package_name}.encoder_verifier_node:main",
        ],
    },
)
