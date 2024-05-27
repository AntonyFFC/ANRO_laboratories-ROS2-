from setuptools import find_packages, setup

package_name = 'lab6PD'

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
    maintainer='antek',
    maintainer_email='01178230@pw.edu.pl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = lab6PD.marker_publisher:main',
        	'marker_sender = lab6PD.marker_sender:main',
        	'robot_pilot = lab6PD.robot_pilot:main',

        ],
    },
)
