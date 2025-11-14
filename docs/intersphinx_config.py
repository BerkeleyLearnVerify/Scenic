"""Shared Intersphinx configuration for Scenic's documentation.

This module defines ``intersphinx_mapping``, which is imported from
``docs/conf.py`` and reused from ``tests/test_docs.py`` to check
connectivity to all configured external documentation sites.
"""

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "matplotlib": ("https://matplotlib.org/stable/", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
    "scipy": ("https://docs.scipy.org/doc/scipy/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
    "pytest": ("https://docs.pytest.org/en/stable/", None),
    "trimesh": ("https://trimesh.org/", None),
}


def iter_intersphinx_urls():
    """Yield the inventory URLs derived from the mapping."""
    for base_url, _ in intersphinx_mapping.values():
        yield base_url.rstrip("/") + "/objects.inv"
