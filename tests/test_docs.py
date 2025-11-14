import os
import subprocess
import sys
from urllib.error import HTTPError, URLError
import urllib.request

import pytest

pytest.importorskip("sphinx")


def _get_intersphinx_urls():
    """Return inventory URLs from docs/intersphinx_config.py."""
    root = os.path.dirname(os.path.dirname(__file__))
    docs_dir = os.path.join(root, "docs")

    sys.path.insert(0, docs_dir)
    try:
        from intersphinx_config import iter_intersphinx_urls

        return list(iter_intersphinx_urls())
    finally:
        sys.path.pop(0)


def _check_intersphinx_connectivity(timeout=3.0):
    """Check Intersphinx inventories before building docs.

    If any inventory URL returns 404, fail as a configuration error.
    If any inventory URL has other HTTP or network errors, skip to avoid flaky CI.
    """
    urls = _get_intersphinx_urls()
    if not urls:
        return

    config_errors = []
    network_errors = []

    for url in urls:
        try:
            urllib.request.urlopen(url, timeout=timeout).close()
        except HTTPError as exc:
            if exc.code == 404:
                config_errors.append(f"{url} (HTTP 404)")
            else:
                network_errors.append(f"{url} (HTTP {exc.code})")
        except URLError as exc:
            network_errors.append(f"{url} ({exc.reason!r})")
        except Exception as exc:
            network_errors.append(f"{url} ({exc!r})")

    if config_errors:
        pytest.fail("intersphinx configuration error(s):\n" + "\n".join(config_errors))

    if network_errors:
        pytest.skip(
            "intersphinx network issue(s), skipping docs build:\n"
            + "\n".join(network_errors)
        )


@pytest.mark.slow
def test_build_docs():
    """Test that the documentation builds and doctests pass."""
    _check_intersphinx_connectivity()

    old_directory = os.getcwd()
    try:
        os.chdir("docs")
        subprocess.run(["make", "clean", "html", "SPHINXOPTS=-W"], check=True)
        subprocess.run(["make", "doctest", "SPHINXOPTS=-W"], check=True)
    finally:
        os.chdir(old_directory)
