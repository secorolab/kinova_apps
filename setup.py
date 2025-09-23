from setuptools import find_packages, setup
from glob import glob

package_name = 'kinova_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vamsi Kalagaturu',
    maintainer_email='vamsikalagaturu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'sort_objects = kinova_apps.sort_objects:main'
        ],
    },
)
