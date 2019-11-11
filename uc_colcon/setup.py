from setuptools import find_packages
from setuptools import setup

package_name = "uc_colcon"

setup(
	name=package_name,
	version="0.0.1",
	packages=[package_name],
	data_files=[
		("share/ament_index/resource_index/packages", [
            "resource/" + package_name
        ]),
		("share/" + package_name, [
            "package.xml"
        ])
	],
	install_requires=["setuptools"],
	zip_safe=True,
	author="Nick Vandemark",
	author_email="rnvandemark@gmail.com",
	maintainer="Nick Vandemark",
	maintainer_email="rnvandemark@gmail.com",
	keywords=["ROS"],
	classifiers=[
		"Intended Audience :: Whoever",
		"License :: Do Whatever",
		"Programming Language :: Python",
		"Topic :: Software Development"
	],
	description="A package that contains other micro-controller code for the peripheral devices in the smart home.",
	license="Have fun, do whatever.",
	tests_require=["pytest"]
)