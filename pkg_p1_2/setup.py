from setuptools import setup, find_packages

package_name = "pkg_p1_2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smyd",
    maintainer_email="inhsroy@hanyang.ac.kr",
    description="Problem 1-2 single package",
    license="TODO",
    entry_points={
        "console_scripts": [
            "p1_2 = pkg_p1_2.p1_2:main",
        ],
    },
)
