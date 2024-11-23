from setuptools import find_packages, setup

package_name = 'a_star'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/a_star_planner.launch.py', 'launch/a_star_planner_with_robot.launch.py']),  # Include both launch files here
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zeiad.ahmed@rwth-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_node = a_star.a_star_planning:main',
        ],
    },
)