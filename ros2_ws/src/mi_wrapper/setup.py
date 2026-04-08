from setuptools import find_packages, setup

package_name = 'mi_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Lego',
    maintainer_email='yo.mama@lego.com',
    description='MI wrapper for DORA',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mi_wrapper = mi_wrapper.mi_wrapper:main'
        ],
    },
)
