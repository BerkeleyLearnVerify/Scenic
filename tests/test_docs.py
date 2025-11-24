import os
import subprocess
import sys
from urllib.error import URLError
from urllib.request import Request, urlopen

import pytest

pytest.importorskip("sphinx")


def _get_intersphinx_urls():
    """Return base URLs from docs/intersphinx_config.py."""
    root = os.path.dirname(os.path.dirname(__file__))
    docs_dir = os.path.join(root, "docs")

    sys.path.insert(0, docs_dir)
    try:
        from intersphinx_config import iter_intersphinx_urls

        return list(iter_intersphinx_urls())
    finally:
        sys.path.pop(0)


def _check_intersphinx_connectivity(timeout=5.0):
    """Check that Intersphinx sites are reachable before building docs.

    Any URL that raises URLError (network or HTTP error) is treated as down;
    if this happens, we skip the docs build test to avoid flaky CI.
    """
    urls = _get_intersphinx_urls()
    if not urls:
        return

    problems = []

    for url in urls:
        # Some docs hosts return 403 for the default Python-urllib user agent.
        req = Request(url, headers={"User-Agent": "Mozilla/5.0"})
        try:
            # Just try to open and then immediately close the response.
            with urlopen(req, timeout=timeout):
                pass
        except (URLError, TimeoutError) as e:
            # Prefer .reason if present (typical for URLError),
            # otherwise use HTTP status code, otherwise repr(e).
            if hasattr(e, "reason"):
                msg = e.reason
            elif hasattr(e, "code"):
                msg = f"HTTP {e.code}"
            else:
                msg = repr(e)
            problems.append(f"{url} ({msg})")

    if problems:
        pytest.skip(
            "Some Intersphinx sites are not reachable; skipping docs build:\n"
            + "\n".join(problems)
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
