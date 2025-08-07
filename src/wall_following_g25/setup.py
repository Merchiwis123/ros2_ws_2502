from setuptools import find_packages, setup

package_name = 'wall_following_g25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_following_g25.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='merchiwis',
    maintainer_email='miguel.merchancano@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MError = wall_following_g25.dist_finder_g25:main',
            'Correr = wall_following_g25.control_g25:main'
        ],
    },
)
