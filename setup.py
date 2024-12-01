import os
from setuptools import setup, find_packages
from glob import glob

PACKAGE_NAME = 'taurasim'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        #('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch/', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/urdf/',  glob('urdf/*.gazebo') + glob('urdf/*.urdf') + glob('urdf/*.xacro') + glob('urdf/*.color')),
        ('share/' + PACKAGE_NAME + '/worlds/', glob('worlds/*.world')),
        ('share/' + PACKAGE_NAME + '/meshes/', glob('meshes/*.stl')),
        ('share/' + PACKAGE_NAME + '/sounds/', glob('sounds/*.wav')),
        ('share/' + PACKAGE_NAME + '/media/materials/scripts', glob('media/materials/scripts/*.material')),
        ('share/' + PACKAGE_NAME + '/media/materials/textures', glob('media/materials/textures/*.png')),
        ('share/' + PACKAGE_NAME + '/models/vss_ball', glob('models/vss_ball/*.config') + glob('models/vss_ball/*.sdf')),
        ('share/' + PACKAGE_NAME + '/models/vss_field', glob('models/vss_field/*.config') + glob('models/vss_field/*.sdf')),
        ('share/' + PACKAGE_NAME + '/models/vss_field/materials/scripts', glob('models/vss_field/materials/scripts/*.material')),
        ('share/' + PACKAGE_NAME + '/models/vss_field/materials/textures', glob('models/vss_field/materials/scripts/*.png')),        
        ('share/' + PACKAGE_NAME + '/models/vss_field/meshes', glob('models/vss_field/meshes/*.stl')),

        ('share/' + PACKAGE_NAME + '/models/vss_field_5', glob('models/vss_field_5/*.config') + glob('models/vss_field_5/*.sdf')),
        ('share/' + PACKAGE_NAME + '/models/vss_field_5/meshes', glob('models/vss_field_5/meshes/*.stl')),
        
        ('share/' + PACKAGE_NAME + '/models/vss_camera', glob('models/vss_camera/*.config') + glob('models/vss_camera/*.sdf')),
        ('share/' + PACKAGE_NAME + '/taurasim/scripts', glob('taurasim/scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Marchesan',
    maintainer_email='lucas1marchesan@gmail.com',
    description='The TauraSim package provides a simulation environment for VSSS robots. \
  It includes configurations for robot models, controllers, and simulation worlds.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_proxy = taurasim.scripts.vision_proxy:main',
            'keyboard_node = taurasim.scripts.keyboard_node:main',
            'spawn_robots = taurasim.scripts.spawn_robots:main',
        ],
    },
)
