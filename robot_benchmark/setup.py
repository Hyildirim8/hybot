from setuptools import setup

package_name = "robot_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.experiments", f"{package_name}.report"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ecza-robotu",
    maintainer_email="dev@ecza.local",
    description="ecza-robotu performans testi ve akademik rapor uretimi",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"robot_benchmark = {package_name}.cli:main",
        ],
    },
)
