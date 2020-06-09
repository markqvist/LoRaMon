import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="loramon",
    version="0.9.5",
    author="Mark Qvist",
    author_email="mark@unsigned.io",
    description="LoRa packet sniffer for RNode hardware",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/markqvist/loramon",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    entry_points= {
        'console_scripts': ['loramon=loramon:main']
    },
    install_requires=['pyserial'],
    python_requires='>=3.5',
)