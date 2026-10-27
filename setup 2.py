import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'capytown_esan'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch y config
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Catedra CapyTown',
    maintainer_email='profesor@esan.edu.pe',
    description='RC-1 La Manzana del Tambo: cinematica, odometria y calibracion de b_eff.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_corrector = capytown_esan.odom_corrector:main',
            'teleop_key = capytown_esan.teleop_key:main',
            'calibrate_beff = capytown_esan.calibrate_beff:main',
            'error_odom = capytown_esan.error_odom:main',
            'fake_robot = capytown_esan.fake_robot:main',
            'run_square = capytown_esan.run_square:main',
        ],
    },
)
