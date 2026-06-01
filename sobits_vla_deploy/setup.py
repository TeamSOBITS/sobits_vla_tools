from setuptools import find_packages, setup
from glob import glob

package_name = 'sobits_vla_deploy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VALENTIN Keith',
    maintainer_email='kvalentincardenas@gmail.com',
    description='ROS packages for SOBITS VLA Deploy.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sobits_vla_deploy = sobits_vla_deploy.sobits_vla_deploy:main',
        ],
    },
)
