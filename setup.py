from setuptools import find_packages, setup

package_name = 'robosyshw2'

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
    maintainer='Hayato Tsukada',
    maintainer_email='szhongtain85@gmail.com',
    description='A package for mouse position tracking and distance calculation',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position = robosyshw2.position:main',
            'distance = robosyshw2.distance:main',
        ],
    },
)
