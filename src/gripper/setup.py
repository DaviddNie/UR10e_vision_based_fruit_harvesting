from setuptools import find_packages, setup

package_name = 'gripper'

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
    maintainer='davidnie',
    maintainer_email='davidnie0418@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'gripper_server = gripper.gripper_server:main',
			'gripper_client = gripper.gripper_client:main',
        ],
    },
)
