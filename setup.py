from setuptools import setup, find_packages


install_requires = [
    'opencv-python',
    'mediapipe',
]


setup(
    name='HandyController',
    version='0.0.0',
    description="Controller with hand pose used in RL environment",
    url="",
    license="MIT",
    packages=find_packages(),
    install_requires=install_requires,
    entry_points={},
)