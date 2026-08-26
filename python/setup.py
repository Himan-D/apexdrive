import os
from setuptools import setup, find_packages

# Read README from root or fallback
long_description = "Universal High-Performance Robotics Actuator & Motor Control Engine"
readme_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "README.md"))
if os.path.exists(readme_path):
    with open(readme_path, "r", encoding="utf-8") as fh:
        long_description = fh.read()

setup(
    name="apexdrive",
    version="1.0.0",
    description="Universal High-Performance Robotics Actuator & Motor Control Engine",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Himan-D",
    author_email="dev@apexdrive.io",
    url="https://github.com/Himan-D/apexdrive",
    project_urls={
        "Bug Tracker": "https://github.com/Himan-D/apexdrive/issues",
        "Source Code": "https://github.com/Himan-D/apexdrive",
        "Documentation": "https://github.com/Himan-D/apexdrive#readme",
    },
    packages=find_packages(),
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: System :: Hardware",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS :: MacOS X",
    ],
    python_requires=">=3.8",
    install_requires=[],
)
