from setuptools import find_packages, setup

package_name = 'rats'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-contrib-python',
        'numpy',
        'pytest',
        'pygame',
        'pyopengl',
        'pyopengl-accelerate',
        'pyrr'
    ],
    zip_safe=True,
    maintainer='bento',
    maintainer_email='bentonsmith@ou.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps = rats.GPSNode:main',
            'camera = rats.CameraNode:main',
            'position = rats.PositionNode:main',
            'tracker = rats.TrackerNode:main',
            'navigation = rats.NavigationNode:main',
            'camera_reader = examples.CameraReader:main',
            'simulation = examples.Simulation:main',
            'viewer = examples.Viewer:main',
            'control = rats.ControlNode:main',
        ],
    },
)
