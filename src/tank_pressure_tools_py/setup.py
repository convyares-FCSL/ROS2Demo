from setuptools import setup

package_name = 'tank_pressure_tools_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carl',
    maintainer_email='you@example.com',
    description='Pressure tools (CSV logger)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pressure_logger = tank_pressure_tools_py.pressure_logger:main',
            'pressure_plotter = tank_pressure_tools_py.pressure_plotter:main',
        ],
    },
)
