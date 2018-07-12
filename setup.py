#!/usr/bin/env python

from setuptools import setup, find_packages

config = {
    'name': 'alvin',
    'version': '0.1.0',
    'license': 'MIT',
    'author': 'Andrew Vardy',
    'author_email': 'av@mun.ca',
    'url': 'http://bots.cs.mun.ca',
    'packages': find_packages(),
    'install_requires': [
        'pyglet>=1.2.4',
        'pymunk>=5.0.0'
    ]
}

setup(**config)
