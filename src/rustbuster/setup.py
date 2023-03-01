from setuptools import setup
import os
from glob import glob

package_name = 'rustbuster'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name + "/launch/"), glob('launch/*_launch*')),
        #(os.path.join('share', package_name + "models/"), glob('models/*/urdf/*urdf*')),
        (os.path.join('share', package_name + "/config/"), glob('config/*')),
        (os.path.join('share', package_name + "/maps/"), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Alemão',
    maintainer_email='malema18@student.aau.dk',
    description='Rust inspection software for Spot robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'talker = rustbuster.publisher_member_function:main',
            #'listener = rustbuster.subscriber_member_function:main',
            'rustbuster_init = rustbuster.rustbuster_main:main',
        ],
    },
)
