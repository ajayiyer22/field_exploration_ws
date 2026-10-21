from setuptools import find_packages, setup

setup(
    name='search_perception',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/search_perception']),
        ('share/search_perception', ['package.xml']),
        ('share/search_perception/models', ['models/yolo26n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='ajayiyer',
    maintainer_email='ajayiyer2397@tamu.edu',
    description='YOLO26 RGB perception using PyTorch CUDA.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'yolo26_detector = search_perception.yolo26_detector:main',
    ]},
)
