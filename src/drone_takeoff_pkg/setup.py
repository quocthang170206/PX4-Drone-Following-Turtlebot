from setuptools import setup

package_name = 'drone_takeoff_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/takeoff_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quocthang',
    maintainer_email='quocthang@todo.todo',
    description='Drone Takeoff Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'takeoff_node = drone_takeoff_pkg.takeoff_node:main'
        ],
    },
)
