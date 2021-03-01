import os
from glob import glob
from setuptools import setup

package_name = 'saisun_driver'

setup(
    data_files=[
        (os.path.join('share',saisun_driver),glob('launch/*.launch.py'))
    ]
)