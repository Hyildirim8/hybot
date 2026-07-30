from setuptools import find_packages, setup

package_name = 'ecza_camera'

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
    description='ESP32-CAM and RPi Camera Module (CSI) receivers for ecza-robotu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'esp32cam_node = ecza_camera.esp32cam_node:main',
            'rpicam_node = ecza_camera.rpicam_node:main',
        ],
    },
)
