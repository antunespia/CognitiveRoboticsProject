from setuptools import find_packages, setup

package_name = 'projeto_rc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install config files so they are available in the package share
        ('share/' + package_name + '/config', [
            'config/waypoints.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guilherme-ver-ssimo',
    maintainer_email='guilherme-ver-ssimo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoints_publisher = projeto_rc.waypoints_publisher:main',
        ],
    },
)
