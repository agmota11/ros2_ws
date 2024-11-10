from setuptools import setup

package_name = 'maadm_practica_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agmota',
    maintainer_email='a.gmota@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_listener = maadm_practica_nodes.keyboard_listener_node:main',
            'turtle_position = maadm_practica_nodes.turtle_position_node:main',
            'trace_controller = maadm_practica_nodes.trace_controller_node:main',
        ],
    },
)
