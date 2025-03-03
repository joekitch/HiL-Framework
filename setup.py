from setuptools import setup, find_packages

setup(
    name="hil-framework",
    version="0.1.0",
    packages=find_packages(include=["hardware", "hardware.*", "utils", "utils.*"]),
    install_requires=[
        "python-can>=4.0.0",
        "cantools>=36.0.0",
        "pytest>=7.0.0",
        "pytest-html>=3.1.1"
    ],
    python_requires=">=3.8",
)