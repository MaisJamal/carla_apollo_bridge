import setuptools

setuptools.setup(
    name="carla_apollo_bridge",
    version="0.0.1",
    author="Mais Jamal",
    author_email="mayssjamal@gmail.com",
    description="Cyber version of Carla ros-bridge.",
    url="https://github.com/MaisJamal/carla_apollo_bridge",
    packages=setuptools.find_packages(),
    python_requires='>=2.7',
    install_requires=[
        'numpy',
        'opencv-python',
        'protobuf',
        'pygame',
        'pyproj',
        'pyyaml',
    ]
)
