from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'sanification'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/sanification.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nicola',
    maintainer_email='nicola@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization=sanification.localization:main',
            'navigation=sanification.navigation:main',
            'sanification=sanification.sanification:main',
            'planner=sanification.planner:main'
            
        ],
    },
)
