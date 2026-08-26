from setuptools import setup, find_packages
import os

readme_path = os.path.join(os.path.dirname(__file__), "..", "README.md")
if not os.path.exists(readme_path):
    readme_path = os.path.join(os.path.dirname(__file__), "README.md")

if os.path.exists(readme_path):
    with open(readme_path, encoding="utf-8") as f:
        long_description = f.read()
else:
    long_description = "Robotics Actuator Control Engine & Inverter SDK"

setup(
    name="apexdrive",
    version="1.2.0",
    description="Robotics Actuator Control Engine & Inverter SDK (FOC / SocketCAN / ROS 2 / STM32G4)",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Himan-D",
    url="https://github.com/Himan-D/apexdrive",
    project_urls={
        "Documentation": "https://github.com/Himan-D/apexdrive#readme",
        "Source": "https://github.com/Himan-D/apexdrive",
        "Tracker": "https://github.com/Himan-D/apexdrive/issues",
    },
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Manufacturing",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development :: Embedded Systems",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
        "Operating System :: Microsoft :: Windows",
    ],
    python_requires=">=3.8",
    install_requires=[],
)
