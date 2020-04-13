# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../src'))
sys.path.insert(0, os.path.abspath('.'))    # for docs-specific code

# Set up paths for Scenic maps to enable importing the world models
import scenic.simulators.gta.map as gta_map
gta_map.mapPath = '../tests/simulators/gta/map.npz'
import scenic.simulators.webots.guideways.intersection as gw_int
gw_int.intersectionPath = '../tests/simulators/webots/guideways/McClintock_DonCarlos_Tempe.json'
import scenic.simulators.webots.road.world as wbt_road_world
wbt_road_world.worldPath = '../tests/simulators/webots/road/simple.wbt'
import scenic.simulators.carla.map as carla_map
carla_map.mapPath = '../tests/simulators/carla/OpenDrive/Town01.xodr'

# -- Project information -----------------------------------------------------

project = 'Scenic'
copyright = '2020, Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia'
author = 'Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
'recommonmark',
'sphinx.ext.autodoc',
'sphinx.ext.autosummary',
'sphinx.ext.napoleon',
'sphinx.ext.viewcode',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

default_role = 'any'

add_module_names = False
autosummary_generate = True
autodoc_inherit_docstrings = False
autodoc_member_order = 'bysource'
napoleon_numpy_docstring = False
napoleon_use_rtype = False

autodoc_default_options = {
    'members': None,
    'private-members': None,
    'show-inheritance': None,
}

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

html_css_files = [
    'custom.css',
]

# -- Extension for correctly displaying Scenic code --------------------------

def setup(app):
    app.connect('viewcode-find-source', handle_find_source)

    return { 'parallel_read_safe': True }

import importlib
from sphinx.pycode import ModuleAnalyzer

def handle_find_source(app, modname):
    try:
        module = importlib.import_module(modname)
    except Exception:
        return None
    if not getattr(module, '_isScenicModule', False):
        return None     # no special handling for Python modules

    # Run usual analysis on the translated source to get tag dictionary
    try:
        analyzer = ModuleAnalyzer.for_module(modname)
        analyzer.find_tags()
    except Exception:
        return None     # bail out; viewcode will try analyzing again but oh well

    # Return original Scenic source, plus tags (line numbers will correspond)
    return module._source, analyzer.tags
