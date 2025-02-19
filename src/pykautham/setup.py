from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

# Define the module name and path
setup(
    name="pykautham",
    version="0.0.1",        # Version of your package
    author="Leopold Palomo-Avellaneda",
    description="Python bindings for Kautham",
    packages=["pykautham"],  # The package directory
    package_data={"pykautham": ["*.so"]},  # Include the .so file
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",  # Adjust based on your target version
)
