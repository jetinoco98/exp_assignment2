from setuptools import setup
import os
from glob import glob 

package_name = 'assignment2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    # Install marker file in the package index
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # Include our package.xml file
    ('share/' + package_name, ['package.xml']),
    # Install URDF files
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.gazebo'))),
    (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
    # Install Rviz Config files
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
    # Install World files
    (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    # Install Launch files
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
    	'waypoint_manager = assignment2.waypoint_manager:main',
    	'inspect = assignment2.inspect:main',
    	'move = assignment2.move:main',
    ],
    },
)
