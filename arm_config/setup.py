from setuptools import find_packages, setup

package_name = 'arm_config'

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
    maintainer_email='bryanmg1010000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_mover_node = arm_config.joint_mover_node:main',
        ],
    },
)
