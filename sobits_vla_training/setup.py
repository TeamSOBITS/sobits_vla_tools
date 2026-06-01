from glob import glob

from setuptools import find_packages, setup

package_name = 'sobits_vla_training'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/policies', glob('config/policies/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VALENTIN Keith',
    maintainer_email='kvalentincardenas@gmail.com',
    description='ROS 2 package for fine-tuning VLA policies on LeRobot datasets.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'train_node = sobits_vla_training.train_node:main',
        ],
    },
)
