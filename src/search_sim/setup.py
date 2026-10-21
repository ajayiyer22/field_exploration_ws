from setuptools import find_packages, setup

package_name = 'search_sim'

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
    description='Gazebo ground-truth localization and RMF door keeping.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ground_truth_localizer = search_sim.ground_truth_localizer:main',
            'door_keeper = search_sim.door_keeper:main',
            'costmap_visualizer = search_sim.costmap_visualizer:main',
            'task_allocation_display = search_sim.task_allocation_display:main',
        ],
    },
)
