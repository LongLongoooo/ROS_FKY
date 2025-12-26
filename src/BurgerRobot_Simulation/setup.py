from setuptools import find_packages, setup

package_name = 'BurgerRobot_Simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_bringup.launch.py']),
        ('share/' + package_name + '/params', ['params/defaults.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phambaolong',
    maintainer_email='phambaolong@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = BurgerRobot_Simulation.controller:main',
            'planner = BurgerRobot_Simulation.planner:main',
            'localizer = BurgerRobot_Simulation.localizer:main',
            'sensor_bridge = BurgerRobot_Simulation.sensor_bridge:main',
            'goal_marker = BurgerRobot_Simulation.goal_marker:main'
        ],
    },
)
