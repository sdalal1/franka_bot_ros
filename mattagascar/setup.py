from setuptools import setup

package_name = 'mattagascar'
submodules = "mattagascar/submodules"
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='demiana',
    maintainer_email='demianabarsoum2027@u.northwestern.edu',
    description='API for interfacing with a PincherX100 and Franka Panda manipulators via moveit2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "iliketomoveitmoveit=mattagascar.iliketomoveitmoveit:main",
        ],
    },
)
