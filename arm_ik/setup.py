import os
from glob import glob
from setuptools import find_packages, setup

package_name = "arm_ik"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Northeastern University Mars Rover Team",
    maintainer_email="northeasternmarsrover@gmail.com",
    description="Package for local inverse kinematics",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_solve_test = test.ik_solve_test:main",
            "local_ik_test = test.local_ik_test:main",
            "local_ik_node = arm_ik.local_ik_node:main",
            "fk_solve_test = test.fk_solve_test:main",
            "arm_status_rviz = test.arm_status_rviz:main",
            "ik_command_rviz = test.ik_command_rviz:main",
        ],
    },
)
