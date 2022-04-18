
import os
import subprocess

import pytest

pytest.importorskip('sphinx')

@pytest.mark.slow
def test_build_docs():
    """Test that the documentation builds.

    We do this in a subprocess since the Sphinx configuration file activates the veneer
    and has other side-effects that aren't reset afterward.
    """
    oldDirectory = os.getcwd()
    try:
        os.chdir('docs')
        subprocess.run(['make', 'clean', 'html', 'SPHINXOPTS=-W'], check=True)
    finally:
        os.chdir(oldDirectory)
