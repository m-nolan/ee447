from gettext import install
import setuptools

with open("README.md") as f:
    long_description = f.read()

install_requires = [
    'numpy',
    'pandas',
    'matplotlib',
    'tqdm',
    'control',      # control system analysis package, mirrors scilab/matlab functions
]

setuptools.setup(
    name='aopy',
    version='1.0.0',
    author='Michael Nolan',
    author_email='manolan@uw.edu',
    description='python code respository for Control Systems I (EE447, University of Washington)',
    long_description=long_description,
    url='https://github.com/m-nolan/ee447',
    packages=setuptools.find_packages(),
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    install_requires=install_requires
)