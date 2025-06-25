from setuptools import find_packages, setup

package_name = 'turtlebot_random_move_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quocthang',
    maintainer_email='quocthang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': ['random_move = turtlebot_random_move_pkg.random_move:main',
        ],
    },
)
