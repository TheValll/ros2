from setuptools import find_packages, setup

package_name = 'py_pkg'

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
    maintainer='valentin',
    maintainer_email='valentin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "minimal_py_node = py_pkg.minimal_node:main",
            "publisher = py_pkg.publisher_node:main",
            "subscriber = py_pkg.subscriber_node:main",
            "server = py_pkg.server_node:main",
            "client = py_pkg.client_node:main",
            "custom_interface = py_pkg.custom_interface_node:main"
        ],
    },
)
