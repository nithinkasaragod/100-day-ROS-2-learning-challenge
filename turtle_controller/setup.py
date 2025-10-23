from setuptools import find_packages, setup

package_name = 'patrol_action_server'

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
    maintainer='abhirami',
    maintainer_email='nithin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'patrol_action_server_exe = patrol_action_server.patrol_action_server:main',
             'patrol_action_client_exe = patrol_action_server.patrol_action_client:main',
        ],
    },
)
