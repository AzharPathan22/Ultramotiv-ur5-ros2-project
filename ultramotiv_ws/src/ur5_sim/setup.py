from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*.*', recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azhar',
    maintainer_email='azhar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
        'launch': [
    	],
    },
)
