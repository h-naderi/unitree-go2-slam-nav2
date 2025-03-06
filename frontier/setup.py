from setuptools import find_packages, setup

package_name = 'frontier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frontier.launch.xml']),
        ('share/' + package_name + '/launch', ['launch/frontier_update.launch.xml']),
        ('share/' + package_name + '/config', ['config/frontier_only.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahulroy',
    maintainer_email='rahulroy2909@gmail.com',
    description='Exploration algorithm based on frontier exploration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = frontier.frontier:main',
            'explore_update = frontier.frontier_update1:main',
            'explore_update2 = frontier.frontier_update2:main',
            'explore_detect = frontier.frontier_detection:main',
        ],
    },
)
