from setuptools import setup

package_name = 'bimanual_isaac'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/sim_bimanual.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/spawn_bimanual.py']),
        ('share/' + package_name + '/config', ['config/robot_description.xacro']),
        ('share/' + package_name + '/urdf', [
            'urdf/allegro_hand.urdf.xacro',
            'urdf/franka_arm.urdf.xacro'
        ]),
        ('share/' + package_name + '/assets', [
            'assets/allegro_hand.usd',
            'assets/franka.usd'
        ]),
        ('share/' + package_name + '/world', [
            'world/table.usd',       
            'world/allegro_hand.usd',
            'world/franka.usd'
        ]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adityanair',
    maintainer_email='aditya.nair0123@gmail.com',
    description='Bimanual Isaac Sim + ROS 2 integration with arms, hands, and sensors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
