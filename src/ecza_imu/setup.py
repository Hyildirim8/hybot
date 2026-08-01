from setuptools import find_packages, setup

package_name = 'ecza_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emre Kabaoğlu',
    maintainer_email='hasan110400@gmail.com',
    description='GY-85 IMU driver for ecza-robotu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = ecza_imu.imu_node:main',
        ],
    },
)
