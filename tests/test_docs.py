import os
import subprocess
from urllib.error import URLError
from urllib.request import Request, urlopen

import pytest

from docs.intersphinx_config import iter_intersphinx_urls

pytest.importorskip("sphinx")


def _check_intersphinx_connectivity(timeout=10.0):
    """Check that Intersphinx sites are reachable before building docs."""
    problems = []

    for url in iter_intersphinx_urls():
        # Some docs hosts don't like the default Python-urllib user-agent.
        req = Request(url, headers={"User-Agent": "Mozilla/5.0"})
        try:
            with urlopen(req, timeout=timeout):
                pass
        except (URLError, TimeoutError) as e:
            # We've seen slow HTTPS responses raise a bare TimeoutError from the
            # SSL layer instead of being wrapped in URLError, so catch both here
            # to avoid flaky failures when docs hosts are slow or unresponsive.
            problems.append(f"{url} ({e})")

    if problems:
        pytest.skip(
            "Some Intersphinx sites are not reachable; skipping docs build:\n"
            + "\n".join(problems)
        )


@pytest.mark.slow
def test_build_docs():
    """Test that the documentation builds, and run doctests.

    We do this in a subprocess since the Sphinx configuration file activates the veneer
    and has other side-effects that aren't reset afterward.
    """
    _check_intersphinx_connectivity()

    oldDirectory = os.getcwd()
    try:
        os.chdir("docs")
        subprocess.run(["make", "clean", "html", "SPHINXOPTS=-W"], check=True)
        subprocess.run(["make", "doctest", "SPHINXOPTS=-W"], check=True)
    finally:
        os.chdir(oldDirectory)
