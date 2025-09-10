from setuptools import setup, find_packages

setup(
    name="mptt-app",
    version="0.2.0",
    packages=find_packages(),
    install_requires=[
        'bleak',
        'numpy',
        # GUI/runtime deps are declared in requirements.txt for dev usage
    ],
    entry_points={
        'console_scripts': [
            'mptt-app = mptt_app.__main__:main',
        ],
    },
    description="MPTT-App (Motorische Precisie en ProprioceptieTest)",
)