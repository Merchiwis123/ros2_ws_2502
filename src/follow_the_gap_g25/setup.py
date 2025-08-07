from setuptools import find_packages, setup

package_name = 'follow_the_gap_g25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/Gap_racer.launch.py']),
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
            'Gap = follow_the_gap_g25.follow_the_gap:main',
        ],
    },
)
