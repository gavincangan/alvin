#!/usr/bin/env python

from setuptools import setup, find_packages

config = {
    'name': 'roboticsintro',
    'version': '0.1.0',
    'license': 'MIT',
    'author': 'Clinton Liddick',
    'author_email': 'clint@clintonliddick.com',
    'url': 'http://www.clintonliddick.com',
    'packages': find_packages(),
    'install_requires': [
        'pyglet>=1.2.4',
        'pymunk>=4.0.0'
    ]
}

setup(**config)
