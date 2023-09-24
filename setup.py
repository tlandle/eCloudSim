from os.path import dirname, realpath
from setuptools import setup, find_packages, Distribution
from ecloud.globals import __version__


def _read_requirements_file():
    """Return the elements in requirements.txt."""
    req_file_path = '%s/requirements.txt' % dirname(realpath(__file__))
    with open(req_file_path) as f:
        return [line.strip() for line in f]


setup(
    name='eCloud',
    version=__version__,
    packages=find_packages(),
    url='https://github.com/tandle/OpenCDA.git#seneca',
    license='MIT',
    author='Jordan Rapp, Tyler Landle',
    author_email='jrapp7@gatech.edu',
    description='A framework for fast developing cooperative driving automation and autonomous '
                'vehicle modules in multi-resolution, distributed, parallel, and asynchronous simulation environment'
                'forked from the UCLA Mobility Lab\'s OpenCDA project',
    long_description=open("README.md", 'r', encoding='utf-8').read(),
    install_requires=_read_requirements_file(),
)
