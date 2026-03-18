from setuptools import setup, find_packages

package_name = "pkg_p3"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smyd",
    maintainer_email="inhsroy@hanyang.ac.kr",
    description="Problem 3 single package",
    license="TODO",
    entry_points={
        "console_scripts": [
            "p3 = pkg_p3.p3:main",
        ],
    },
)
