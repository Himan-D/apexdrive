from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
import os
import sys

readme_path = os.path.join(os.path.dirname(__file__), "..", "README.md")
if not os.path.exists(readme_path):
    readme_path = os.path.join(os.path.dirname(__file__), "README.md")

if os.path.exists(readme_path):
    with open(readme_path, encoding="utf-8") as f:
        long_description = f.read()
else:
    long_description = "Robotics Actuator Control Engine & Inverter SDK"

class OptionalBuildExt(build_ext):
    def build_extension(self, ext):
        try:
            super().build_extension(ext)
        except Exception as e:
            print(f"Warning: Building native C++ extension failed ({e}). Falling back to pure Python client.")

# Define C++ Accelerated Extension
extra_compile_args = ["-std=c++20", "-O3"]
if sys.platform == "win32":
    extra_compile_args = ["/std:c++20", "/O2"]

ext_modules = [
    Extension(
        "apexdrive._apexdrive_c",
        sources=["src/apexdrive_c_module.cpp"],
        include_dirs=["../include"],
        extra_compile_args=extra_compile_args,
        language="c++"
    )
]

setup(
    name="apexdrive",
    version="1.3.0",
    description="Robotics Actuator Control Engine & Inverter SDK (FOC / SocketCAN / ROS 2 / STM32G4 / Isaac Sim)",
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
    ext_modules=ext_modules,
    cmdclass={"build_ext": OptionalBuildExt},
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
