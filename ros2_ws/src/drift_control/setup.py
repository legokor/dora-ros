import os
from glob import glob
from setuptools import setup

package_name = 'drift_control'

setup(
    name=package_name,
    version='4.2.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='anyad',
    author_email='yo.mama@lego.com',
    maintainer='John Lego',
    maintainer_email='yo.mama@lego.com',
    keywords=['horok', 'kereszt'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: Apache',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Teleoperation from keyboard',
    license='Apache',
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': [
            'teleop_key = drift_control.teleop_key:main'
        ],
    },
)