from setuptools import setup

package_name = 'orunav2_gazebo_spawner'

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
    maintainer='Paolo Forte',
    maintainer_email='paolo.forte@oru.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_gazebo_spawner = orunav2_gazebo_spawner.nav2_gazebo_spawner:main',
        ],
    },
)
