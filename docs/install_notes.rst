.. _install_notes:

Notes on Installing Scenic
==========================

This page describes common issues with installing Scenic and suggestions for fixing them.

All Platforms
--------------

Missing Python Version
++++++++++++++++++++++

If during installation (with either :command:`pip` or Poetry) you get an error saying that your machine does not have a compatible version, this means that you do not have Python 3.7 or later on your PATH.
Install a newer version of Python, either directly from the `Python website <https://www.python.org/downloads/>`_ or using `pyenv <https://github.com/pyenv/pyenv>`_ (e.g. running :command:`pyenv install 3.10.4`).
If you install it somewhere that is not on your PATH (so running :command:`python --version` doesn't give you the correct version), you'll need to run :command:`poetry env use /full/path/to/python` before running :command:`poetry install`.

Scene Schematics Don't Appear
+++++++++++++++++++++++++++++

If no window appears when you ask Scenic to generate and display a scene (as in the example commands in :doc:`quickstart`), this means that Matplotlib has no `interactive backend <https://matplotlib.org/stable/users/explain/backends.html>`_ installed.
On Linux, try installing the ``python3-tk`` package (e.g. :command:`sudo apt-get install python3-tk`).

Missing SDL
+++++++++++

If you get an error about `SDL <https://www.libsdl.org/>`_ being missing, you may need to install it.
On Linux (or Windows with :ref:`WSL <wsl>`), install the ``libsdl2-dev`` package (e.g. :command:`sudo apt-get install libsdl2-dev`); on macOS, if you use `Homebrew <https://brew.sh/>`_ you can run :command:`brew install sdl2`.
For other platforms, see the SDL website.

Using a Local Scenic Version with VerifAI
+++++++++++++++++++++++++++++++++++++++++

If you are using Scenic as part of the `VerifAI`_ toolkit, the VerifAI installation process will automatically install Scenic from PyPI.
However, if you need to use your own fork of Scenic or some features which have not yet been released on PyPI, you will need to install Scenic manually in VerifAI's virtual environment.
The easiest way to do this is as follows:

1. Install VerifAI in a virtual environment of your choice (or one created automatically by Poetry).
2. Enter the virtual environment (e.g. :command:`poetry shell`).
3. Change directory to your clone of the Scenic repository.
4. Run :command:`pip install -e .` (this requires at least pip version 21.3).

You can test that this process has worked correctly by going back to the VerifAI repo and running the Scenic part of its test suite with :command:`pytest tests/test_scenic.py`.

.. note::

	Installing Scenic in this way bypasses Poetry's dependency resolution for VerifAI.
	If your local version of Scenic requires different versions of some of VerifAI's dependencies, you may get errors from :command:`pip` about dependency conflicts.
	Such errors do not actually prevent Scenic from being installed; however you may get unexpected behavior from VerifAI at runtime.
	If you are developing forks of Scenic and VerifAI, a more stable approach would be to modify VerifAI's :file:`pyproject.toml` to point to your fork of Scenic instead of the ``scenic`` package on PyPI.

.. _VerifAI: https://github.com/BerkeleyLearnVerify/VerifAI

Windows
-------

.. _wsl:

Using WSL
+++++++++

For greatest ease of installation, we recommend using the `Windows Subsystem for Linux <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_ (WSL, a.k.a. "Bash on Windows") on Windows 10 and newer.
Poetry can be installed on WSL in the same way as on Linux: see the instructions `here <https://python-poetry.org/docs/master/#installing-with-the-official-installer>`__.

It is possible to run Scenic natively on Windows; however, in the past there have been issues with some of Scenic's dependencies either not providing wheels for Windows or requiring manual installation of additional libraries.

Problems building Shapely
+++++++++++++++++++++++++

In the past, the ``shapely`` package did not install properly on Windows.
If you encounter this issue, try installing it manually following the instructions `here <https://github.com/Toblerity/Shapely#built-distributions>`__.
