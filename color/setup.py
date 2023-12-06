from setuptools import find_packages, setup

package_name = "color"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sdalal",
    maintainer_email="shaildalal2024@u.northwestern.edu",
    description="Packge detects the position of colors on the palette and broadcasts them to tf",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["colordetection = color.colordetection:main"],
    },
)
