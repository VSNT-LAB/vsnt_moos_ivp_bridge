from setuptools import setup

package_name = 'vsnt_moos_ivp_bridge'

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
    maintainer='dueiras',
    maintainer_email='dueiras@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moos2ros = vsnt_moos_ivp_bridge.moos_ivp_to_ros:main',
            'nav_plotter = vsnt_moos_ivp_bridge.nav_plotter:main',
        ],
    },
)
