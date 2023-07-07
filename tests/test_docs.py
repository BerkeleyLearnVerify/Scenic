import os
import socket
import subprocess

import pytest

pytest.importorskip("sphinx")


@pytest.mark.slow
def test_build_docs():
    """Test that the documentation builds, and run doctests.

    We do this in a subprocess since the Sphinx configuration file activates the veneer
    and has other side-effects that aren't reset afterward.
    """
    try:
        socket.getaddrinfo("docs.python.org", 80)
    except OSError:
        pytest.skip("cannot connect to python.org for Intersphinx")

    oldDirectory = os.getcwd()
    try:
        os.chdir("docs")
        subprocess.run(["make", "clean", "html", "SPHINXOPTS=-W"], check=True)
        subprocess.run(["make", "doctest", "SPHINXOPTS=-W"], check=True)
    finally:
        os.chdir(oldDirectory)
