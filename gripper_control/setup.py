from setuptools import find_packages, setup

package_name = 'gripper_control'

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
    maintainer='pav',
    maintainer_email='pavlin.rodrigues@student.uts.edu.au',
    description='Gripper control node for RG2 fake‐hardware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # <executable name> = <module path>:<callable>
            'gripper_control = gripper_control.gripper_control:main',
        ],
    },
)

