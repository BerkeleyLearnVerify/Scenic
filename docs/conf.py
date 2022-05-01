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

# Hack to set global parameters needed to import the driving domain models
import scenic.syntax.veneer as veneer
veneer.activate(paramOverrides=dict(
    map='../tests/formats/opendrive/maps/opendrive.org/CulDeSac.xodr',
    carla_map='blah',
    lgsvl_map='blah',
))
import scenic.simulators.carla.model
import scenic.simulators.lgsvl.model

# -- Project information -----------------------------------------------------

project = 'Scenic'
copyright = '2022, Daniel J. Fremont'
author = 'Daniel J. Fremont, Edward Kim, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.autosummary',
'sphinx.ext.coverage',
'sphinx.ext.napoleon',
'sphinx.ext.viewcode',
'sphinx.ext.intersphinx',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

default_role = 'any'
#nitpicky = True

add_module_names = False
autosummary_generate = True
autodoc_inherit_docstrings = False
autodoc_member_order = 'bysource'
autodoc_mock_imports = ['carla', 'lgsvl']
autodoc_typehints = 'description'
autodoc_type_aliases = {
    'Vectorlike': '`scenic.domains.driving.roads.Vectorlike`',
}
napoleon_numpy_docstring = False
napoleon_use_rtype = False
napoleon_use_ivar = True

autodoc_default_options = {
    'members': None,

    # include members starting with underscores by default;
    # we'll install an extension below to skip those with :meta private:
    'private-members': None,

    'show-inheritance': None,
}

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'matplotlib': ('https://matplotlib.org/stable/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
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

# -- Monkeypatch to resolve ambiguous references -----------------------------

from sphinx.domains.python import PythonDomain
orig_find_obj = PythonDomain.find_obj

def find_obj(self, env, modname, classname, name, type, searchmode,
             *args, **kwargs):
    results = orig_find_obj(self, env, modname, classname, name, type,
                            searchmode, *args, **kwargs)
    if len(results) <= 1:
        return results

    def score(res):
        name = res[0]
        if name.startswith('scenic.syntax.veneer'):
            return -1
        elif modname is None:
            return 0
        else:
            # score by length of common prefix
            for i, (c, d) in enumerate(zip(modname, name)):
                if c != d:
                    break
            return i

    scores = [score(res) for res in results]
    highScore = max(scores)
    highest = [
        res for res, score in zip(results, scores)
        if score == highScore
    ]
    return highest

PythonDomain.find_obj = find_obj

# -- Monkeypatch to improve formatting of class/instance attributes ----------

# This is based on code suggested by Michael Goerz at:
# https://michaelgoerz.net/notes/extending-sphinx-napoleon-docstring-sections.html

from sphinx.ext.napoleon.docstring import GoogleDocstring

def parse_attributes_section(self, section):
    return self._format_fields('Attributes', self._consume_fields())
GoogleDocstring._parse_attributes_section = parse_attributes_section

def parse_class_attributes_section(self, section):
    return self._format_fields('Class Attributes', self._consume_fields())
GoogleDocstring._parse_class_attributes_section = parse_class_attributes_section

def parse_properties_section(self, section):
    return self._format_fields('Properties', self._consume_fields())
GoogleDocstring._parse_properties_section = parse_properties_section

def parse_global_params_section(self, section):
    return self._format_fields('Global Parameters', self._consume_fields())
GoogleDocstring._parse_global_params_section = parse_global_params_section

orig_parse = GoogleDocstring._parse
def parse(self):
    self._sections['global parameters'] = self._parse_global_params_section
    self._sections['class attributes'] = self._parse_class_attributes_section
    self._sections['properties'] = self._parse_properties_section
    orig_parse(self)
GoogleDocstring._parse = parse

# -- Monkeypatch to list private members after public ones -------------------

from sphinx.ext.autodoc import Documenter

orig_sort_members = Documenter.sort_members
def sort_members(self, documenters, order):
    documenters = orig_sort_members(self, documenters, order)
    def key(entry):
        parts = entry[0].name.split('::')
        if len(parts) == 2:
            name = parts[-1]
        else:
            assert len(parts) == 1
            name = parts[0].split('.')[-1]
        return name.startswith('_')
    documenters.sort(key=key)
    return documenters
Documenter.sort_members = sort_members

# -- Monkeypatch to hide private base classes --------------------------------
# TODO use autodoc-process-bases instead (new in Sphinx 4.1)

from sphinx.ext.autodoc import ClassDocumenter

orig_add_directive_header = ClassDocumenter.add_directive_header
def add_directive_header(self, sig):
    oldBases = self.object.__bases__
    if oldBases != (object,) and self.options.show_inheritance:
        newBases = [base for base in oldBases if not base.__name__.startswith('_')]
        if newBases:
            self.object.__bases__ = tuple(newBases)
            orig_add_directive_header(self, sig)
            self.object.__bases__ = oldBases
            return
    oldSI = self.options.show_inheritance
    self.options.show_inheritance = False
    orig_add_directive_header(self, sig)
    self.options.show_inheritance = oldSI
ClassDocumenter.add_directive_header = add_directive_header

# -- Monkeypatch to improve formatting of certain values ---------------------

import scenic
import sphinx.ext.autodoc

orig_object_description = sphinx.ext.autodoc.object_description
def object_description(obj):
    if obj is scenic.core.regions.nowhere or obj is scenic.core.regions.everywhere:
        return str(obj)
    elif isinstance(obj, (scenic.core.regions.Region,
                          scenic.core.vectors.VectorField,
                          scenic.domains.driving.roads.Network)):
        raise ValueError    # prevent rendering of this value
    else:
        return orig_object_description(obj)
sphinx.ext.autodoc.object_description = object_description

# -- Extension for correctly displaying Scenic code and skipping internals ---

from scenic.syntax.pygment import ScenicLexer

def setup(app):
    app.connect('viewcode-find-source', handle_find_source)
    app.connect('autodoc-process-signature', handle_process_signature)
    app.connect('autodoc-skip-member', handle_skip_member)

    # Run our own reference resolver before the standard one, to resolve refs to
    # Scenic classes differently in the "Scenic Internals" section than elsewhere
    app.add_post_transform(ScenicRefResolver)

    # To allow either plural or singular forms to be looked up in the glossary:
    app.connect('missing-reference', handle_missing_reference)

    # for some reason, the Pygments entry point doesn't work on ReadTheDocs;
    # so we register the custom lexer here
    app.add_lexer('scenic', ScenicLexer)

    return { 'parallel_read_safe': True }

import importlib
from sphinx.pycode import ModuleAnalyzer
from sphinx.util.docstrings import extract_metadata

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

def handle_process_signature(app, what, name, obj, options, signature, ret_anno):
    if (what == 'class'
        and issubclass(obj, scenic.core.object_types.Constructible)
        and obj is not scenic.core.object_types.Constructible):
        return ('(<specifiers>)', None)
    else:
        return None

def handle_skip_member(app, what, name, obj, skip, options):
    if not skip:
        doc = getattr(obj, '__doc__')
        if doc and 'private' in extract_metadata(doc):
            return True
    return None

import inflect
engine = inflect.engine()

def handle_missing_reference(app, env, node, contnode):
    """If a glossary term can't be found, try the opposite plurality."""
    if node.get('refdomain') != 'std':
        return None
    if node['reftype'] != 'term':
        return None
    prefix, sep, last_word = node['reftarget'].rpartition(' ')
    alternative = engine.singular_noun(last_word)
    if alternative is False:   # word is not a plural noun
        alternative = engine.plural(last_word)
    target = prefix + sep + alternative
    stddomain = env.domains['std']
    newnode = stddomain.resolve_xref(env, node.get('refdoc'), app.builder,
                                     'term', target, node, contnode)
    return newnode

from sphinx.transforms.post_transforms import ReferencesResolver

class ScenicRefResolver(ReferencesResolver):
    default_priority = ReferencesResolver.default_priority - 2

    # The following is copied from ReferencesResolver.resolve_anyref with minor changes
    def resolve_anyref(self, refdoc, node, contnode):
        stddomain = self.env.get_domain('std')
        target = node['reftarget']
        stdresults = []
        # first, try resolving as :doc:
        doc_ref = stddomain.resolve_xref(self.env, refdoc, self.app.builder,
                                         'doc', target, node, contnode)
        if doc_ref:
            stdresults.append(('doc', doc_ref))
        # next, do the standard domain (makes this a priority)
        stdresults.extend(stddomain.resolve_any_xref(self.env, refdoc, self.app.builder,
                                                     target, node, contnode))
        domresults = []
        for domain in self.env.domains.values():
            if domain.name == 'std':
                continue  # we did this one already
            try:
                domresults.extend(domain.resolve_any_xref(self.env, refdoc, self.app.builder,
                                                          target, node, contnode))
            except NotImplementedError:
                # the domain doesn't yet support the new interface
                # we have to manually collect possible references (SLOW)
                for role in domain.roles:
                    res = domain.resolve_xref(self.env, refdoc, self.app.builder,
                                              role, target, node, contnode)
                    if res and len(res) > 0 and isinstance(res[0], nodes.Element):
                        domresults.append(('%s:%s' % (domain.name, role), res))
        # now, see how many matches we got...
        results = stdresults + domresults
        if not results:
            return None
        if len(stdresults) > 1 or len(domresults) > 1:
            def stringify(name: str, node: Element) -> str:
                reftitle = node.get('reftitle', node.astext())
                return ':%s:`%s`' % (name, reftitle)
            candidates = ' or '.join(stringify(name, role) for name, role in results)
            logger.warning(__('more than one target found for \'any\' cross-'
                              'reference %r: could be %s'), target, candidates,
                           location=node)
        if stdresults and domresults:
            # disambiguate based on whether this is internal documentation or not
            results = domresults if refdoc.startswith('modules/') else stdresults
        res_role, newnode = results[0]
        # Override "any" class with the actual role type to get the styling
        # approximately correct.
        res_domain = res_role.split(':')[0]
        if (len(newnode) > 0 and
                isinstance(newnode[0], nodes.Element) and
                newnode[0].get('classes')):
            newnode[0]['classes'].append(res_domain)
            newnode[0]['classes'].append(res_role.replace(':', '-'))
        return newnode

# -- Big monkeypatch to fix bug in autosummary (temporarily) -----------------

from sphinx.pycode import ModuleAnalyzer, PycodeError
import sphinx.ext.autosummary.generate as as_gen

orig_generate_autosummary_content = as_gen.generate_autosummary_content
orig_scan = as_gen.ModuleScanner.scan

def generate_autosummary_content(name, obj, parent,
                                 template, template_name,
                                 imported_members, app,
                                 recursive, context,
                                 modname = None, qualname = None):
    members = dir(obj)
    attrs = set()
    try:
        analyzer = ModuleAnalyzer.for_module(name)
        attr_docs = analyzer.find_attr_docs()
        for namespace, attr_name in attr_docs:
            if namespace == '' and attr_name in members:
                attrs.add(attr_name)
    except PycodeError:
        pass    # give up if ModuleAnalyzer fails to parse code

    def scan(self, imported_members):
        scanned = set(orig_scan(self, imported_members))
        allMembers = []
        for name in dir(self.object):
            if name in scanned or name in attrs:
                allMembers.append(name)
        return allMembers
    as_gen.ModuleScanner.scan = scan

    return orig_generate_autosummary_content(name, obj, parent,
                                             template, template_name,
                                             imported_members, app,
                                             recursive, context,
                                             modname=modname, qualname=qualname)

as_gen.generate_autosummary_content = generate_autosummary_content

# -- Monkeypatch adding a :sampref: role combining :samp: and :ref: ----------
# (necessary since ReST does not allow nested inline markup)

from docutils import nodes
import docutils.parsers.rst.roles
from sphinx.roles import XRefRole, EmphasizedLiteral

class LiteralXRefRole(XRefRole, EmphasizedLiteral):
    def create_xref_node(self):
        self.refdomain, self.reftype = 'std', 'sampref'
        elem, _ = super().create_xref_node()
        node = elem[0]
        del node[0]
        children = self.parse(self.title)
        node += nodes.literal(self.title, '', *children,
                             role='samp', classes=['xref', 'ref', 'samp'])
        return [node], []

role = LiteralXRefRole(lowercase=True, innernodeclass=nodes.inline,
                       warn_dangling=True)

docutils.parsers.rst.roles.register_local_role('sampref', role)

from sphinx.domains.std import StandardDomain
old_resolve_xref = StandardDomain.resolve_xref

def resolve_xref(self, env, fromdocname, builder, typ, target, node, contnode):
    if typ == 'sampref':
        newnode = old_resolve_xref(self, env, fromdocname, builder, 'ref', target,
                                   node, contnode)
        if not newnode:
            return newnode
        del newnode[0]
        inner = node[0]
        inner['classes'] = ['std', 'std-ref']
        newnode += inner
    else:
        newnode = old_resolve_xref(self, env, fromdocname, builder, typ, target,
                                   node, contnode)
    return newnode

StandardDomain.resolve_xref = resolve_xref
