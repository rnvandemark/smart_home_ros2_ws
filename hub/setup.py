from setuptools import find_packages
from setuptools import setup

package_name = "hub"

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
        ]),
		("lib/" + package_name, [
            "hub/GUI.py",
            "hub/SmartHomeHubController.py",
            "hub/SmartHomeHubNode.py",
            "hub/AudioHandler.py"
        ]),
		("lib/" + package_name + "/resources", [
            "resource/kyoto.png"
        ]),
		("lib/" + package_name + "/resources/traffic_lights", [
			"resource/traffic_lights/traffic_light_none.png",
			"resource/traffic_lights/traffic_light_green.png",
			"resource/traffic_lights/traffic_light_yellow.png",
			"resource/traffic_lights/traffic_light_red.png",
			"resource/traffic_lights/traffic_light_all.png"
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
	description="The control center for my smart home.",
	license="Have fun, do whatever.",
	tests_require=["pytest"],
	entry_points={
		"console_scripts": [
			"hub_main = hub.Runner:main",
		]
	}
)