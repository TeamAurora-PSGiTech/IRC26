from setuptools import find_packages, setup

package_name = 'arm_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msgs', ['msgs/Arm.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandy',
    maintainer_email='sandytheadventurer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "interface = arm_ctrl.interface:main",
            "teleop = arm_ctrl.teleop:main",
        ],
    },
)
