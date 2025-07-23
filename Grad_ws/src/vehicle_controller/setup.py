from setuptools import find_packages, setup
from glob import glob

package_name = 'vehicle_controller'

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
    install_requires=[
        'setuptools',
        'pynput',
        'matplotlib',
        'flask',
        'flask_socketio',
        'eventlet',
        'numpy',
        'scikit-fuzzy',
        'networkx',
        ],
    zip_safe=True,
    maintainer='Morad Sayed',
    maintainer_email='Morad.S.Singer@gmail.com',
    description='TODO: Package description',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "adas = vehicle_controller.vehicle_controller:main"
        ],
    },
)
