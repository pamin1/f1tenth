from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'f1tenth'

setup(
    name='f1tenth',
    version='0.1.0',
    packages=find_packages(),  # This should pick up the inner 'f1tenth' package
    install_requires=['setuptools'],
    zip_safe=True,
    author='prachit',
    author_email='prachitamin12@gmail.com',
    description='MPCC controller for RC cars',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test = f1tenth.test:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
)
