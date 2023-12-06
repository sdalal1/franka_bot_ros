from setuptools import find_packages, setup

package_name = 'take_picture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/take_picture.launch.xml',
                                   'config/pcl.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naderahmed',
    maintainer_email='naderahmed2024@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'picture_node = take_picture.picture_node:main'
        ],
    },
)
