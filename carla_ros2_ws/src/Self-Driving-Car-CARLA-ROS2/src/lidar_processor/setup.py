from setuptools import setup

package_name = 'lidar_processor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='LiDAR point cloud processing and analysis nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_analyzer    = lidar_processor.lidar_analyzer:main',
            'ground_remover    = lidar_processor.ground_remover:main',
            'object_clusterer  = lidar_processor.object_clusterer:main',
            'detector_validator = lidar_processor.detector_validator:main',
            'lidar_preprocessor = lidar_processor.lidar_preprocessor:main',
        ],
    },
)
