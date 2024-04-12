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


# -- Project information -----------------------------------------------------

project = 'VerifAI'
copyright = '2022, Daniel J. Fremont, Shromona Ghosh, Edward Kim, and Sanjit A. Seshia.'
author = 'Tommaso Dreossi, Daniel J. Fremont, Shromona Ghosh, Edward Kim, Hadi Ravanbaksh, Marcell Vazquez-Chanlatte, and Sanjit A. Seshia'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
	'recommonmark',
	'sphinx.ext.autodoc',
	'sphinx.ext.autosummary',
	'sphinx.ext.doctest',
	'sphinx.ext.intersphinx',
	'sphinx.ext.napoleon',
	'sphinx.ext.viewcode',
	'sphinx_tabs.tabs',
]

source_suffix = {
	'.rst': 'restructuredtext',
	'.md': 'markdown',
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '.pytest_cache']

default_role = 'any'

add_module_names = False
autosummary_generate = True
autodoc_inherit_docstrings = False
autodoc_member_order = 'bysource'
napoleon_numpy_docstring = False
napoleon_use_rtype = False
napoleon_use_ivar = True

autodoc_default_options = {
    'members': None,
}

intersphinx_mapping = {
	'python': ('https://docs.python.org/3', None),
	'scenic': ('https://scenic-lang.readthedocs.io/en/latest/', None),
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
