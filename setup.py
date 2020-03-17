"""This is the setup file"""

from setuptools import setup, find_packages

with open('README.md', 'r') as r:
    long_description = r.read()

setup(name='scenic',
      version='1.0.0b2',
      install_requires=[
          'numpy',
          'scipy',
          'matplotlib',
          'antlr4-python3-runtime',
          'opencv-python',
          'pillow',
          'shapely', # had to separately install via conda install shapely
          'Polygon3',
          'dotmap',
      ],
      extras_require={
        'pyproj': ['pyproj'], # issue on Windows with Anaconda
      },
      python_requires='>=3.6',
      packages=find_packages('src'),
      package_dir={'': 'src'},
      package_data={
        '': ['*.sc'],
      },

      author='Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia',
      author_email='dfremont@berkeley.edu',
      description='The Scenic scenario description language.',
      long_description=long_description,
      long_description_content_type='text/markdown',

      classifiers=[
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Development Status :: 4 - Beta',
      ]
)
