from setuptools import find_packages, setup

package_name = "demo_scripts"

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
    maintainer="Team 4",
    maintainer_email="174347826+bt-nav@users.noreply.github.com",
    description="Scripts for the promo video and demonstration.",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "demo_drop = demo_scripts.demo_drop:main",
            "demo_pick_vision = demo_scripts.demo_pick_vision:main",
            "demo_pick = demo_scripts.demo_pick:main",
            "demo_scan = demo_scripts.demo_scan:main",
        ],
    },
)
