from setuptools import find_packages, setup

package_name = 'listen_apriltags'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/aprilTags.launch.xml',
                                   'config/tags.yaml',
                                   'config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csmith',
    maintainer_email='courtney77smith77@gmail.com',
    description='creates transforms using apriltags',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = listen_apriltags.listening:main'
        ],
    },
)
