import setuptools


setuptools.setup(
    name='skyengine',
    packages=['skyengine'],
    version='3.1.0',
    description='Skynet avionics library',
    author='Andrew Brooks',
    author_email='abrooks@rice.edu',
    url='https://code.skynet.engineering/diffusion/SKYENGINE',
    install_requires=[
        'aenum==2.0.8',
        'dronekit==2.9.1',
        'dronekit-sitl==3.2.0',
        'pymavlink==2.2.5',
        'redis==2.10.6',
    ],
    keywords=[],
    classifiers=[],
)
