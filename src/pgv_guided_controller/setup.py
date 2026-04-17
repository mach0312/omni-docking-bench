from setuptools import find_packages, setup

package_name = 'pgv_guided_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pgv_controller.launch.py',
            ]),
        ('share/' + package_name + '/config', [
            'config/pgv_controller.params.yaml',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaerak',
    maintainer_email='sjr9017@khu.ac.kr',
    description='PGV-based coordinate controller for holonomic/non-holonomic platforms.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pgv_controller = pgv_guided_controller.pgv_controller_node:main',
        ],
    },
)
