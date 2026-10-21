from setuptools import find_packages, setup

package_name = 'exploration_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='ajayiyer',
    maintainer_email='ajayiyer2397@tamu.edu',
    description='Shared occupancy grid and size-dependent per-robot costmaps.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'exploration_map = exploration_map.exploration_map:main',
        ],
    },
)
