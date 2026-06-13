from glob import glob
from setuptools import find_packages, setup

package_name = 'sobits_vla_rosbag_conversion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config',  glob('config/*.yaml')),
        (f'share/{package_name}/lerobotdataset', glob('lerobotdataset/.gitkeep')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VALENTIN Keith',
    maintainer_email='kvalentincardenas@gmail.com',
    description='ROS packages for SOBITS VLA Rosbag Conversion.',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros2bag_to_lerobotdataset = sobits_vla_rosbag_conversion.ros2bag_to_lerobotdataset:main',
        ],
    },
)
