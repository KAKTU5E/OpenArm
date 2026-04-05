from setuptools import find_packages, setup

package_name = 'angle_publisher_subscriber'

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
    maintainer='openmutt',
    maintainer_email='supermaxpizz2006@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'angle_publisher = angle_publisher_subscriber.angle_publisher:main',
            'CubeMars_Angle_Publisher = angle_publisher_subscriber.cube_mars_angle_publisher:main',
            'angle_subscriber = angle_publisher_subscriber.angle_subscriber:main',
            'angle_routine = angle_publisher_subscriber.angle_routine:main',
            'angle_can_bridge = angle_publisher_subscriber.angle_can_bridge:main',
        ],
    },
)
