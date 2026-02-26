from setuptools import find_packages, setup
from glob import glob

package_name = 'vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karim, Morad Sayed',
    maintainer_email='kraafat.m51@gmail.com, Morad.S.Singer@gmail.com',
    description='TODO: Package description',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuators = vehicle_interface.Actuator_node:main',
            'lcd = vehicle_interface.lcd_UI.UI:main'
        ],
    },
)
