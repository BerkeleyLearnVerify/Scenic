.. _install_notes:

Notes on Installing Scenic
==========================

This page describes common issues with installing Scenic and suggestions for fixing them.

All Platforms
--------------

Missing Python Version
++++++++++++++++++++++

If when running :command:`pip` you get an error saying that your machine does not have a compatible version, this means that you do not have Python 3.8 or later on your PATH.
Install a newer version of Python, either directly from the `Python website <https://www.python.org/downloads/>`_ or using `pyenv <https://github.com/pyenv/pyenv>`_ (e.g. running :command:`pyenv install 3.10.4`).
Then use that version of Python when creating a virtual environment before installing Scenic.

"setup.py" not found
++++++++++++++++++++

This error indicates that you are using too old a version of ``pip``: you need at least version 21.3.
Run :command:`python -m pip install --upgrade pip` to upgrade.

Dependency Conflicts
++++++++++++++++++++

If you install Scenic using ``pip``, you might see an error message like the following:

	ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.

This error means that in order to install Scenic, ``pip`` had to break the dependency constraints of some package you had previously installed (the error message will indicate which one).
So while Scenic will work correctly, something else may now be broken.
This won't happen if you install Scenic into a fresh virtual environment.

Cannot Find Scenic
++++++++++++++++++

If when running the :command:`scenic` command you get a "command not found" error, or when trying to import the ``scenic`` module you get a `ModuleNotFoundError`, then Scenic has not been installed where your shell or Python respectively can find it.
The most likely problem is that you installed Scenic for one copy of Python but are now using a different one: for example, if you installed Scenic in a Python virtual environment (which we highly recommend), you may have forgotten to activate that environment, and so are using your system Python instead.
See the `virtual environment tutorial <https://docs.python.org/3/tutorial/venv.html>`_ for instructions.

Scene Schematics Don't Appear (2D)
++++++++++++++++++++++++++++++++++

If no window appears when you ask Scenic to generate and display a scene using the :option:`--2d` flag (as in the example commands in :doc:`quickstart`), this means that Matplotlib has no `interactive backend <https://matplotlib.org/stable/users/explain/backends.html>`_ installed.
On Linux, try installing the ``python3-tk`` package (e.g. :command:`sudo apt-get install python3-tk`).

Missing SDL
+++++++++++

If you get an error about `SDL <https://www.libsdl.org/>`_ being missing, you may need to install it.
On Linux (or Windows with :ref:`WSL <wsl>`), install the ``libsdl2-dev`` package (e.g. :command:`sudo apt-get install libsdl2-dev`); on macOS, if you use `Homebrew <https://brew.sh/>`__ you can run :command:`brew install sdl2`.
For other platforms, see the SDL website.

Using a Local Scenic Version with VerifAI
+++++++++++++++++++++++++++++++++++++++++

If you are using Scenic as part of the `VerifAI`_ toolkit, the VerifAI installation process will automatically install Scenic from PyPI.
However, if you need to use your own fork of Scenic or some features which have not yet been released on PyPI, you will need to install Scenic manually in VerifAI's virtual environment.
The easiest way to do this is as follows:

1. Install VerifAI in a virtual environment of your choice.
2. Activate the virtual environment.
3. Change directory to your clone of the Scenic repository.
4. Run :command:`pip install -e .`

You can test that this process has worked correctly by going back to the VerifAI repo and running the Scenic part of its test suite with :command:`pytest tests/test_scenic.py`.

.. note::

	Installing Scenic in this way bypasses dependency resolution for VerifAI.
	If your local version of Scenic requires different versions of some of VerifAI's dependencies, you may get errors from :command:`pip` about dependency conflicts.
	Such errors do not actually prevent Scenic from being installed; however you may get unexpected behavior from VerifAI at runtime.
	If you are developing forks of Scenic and VerifAI, a more stable approach would be to modify VerifAI's :file:`pyproject.toml` to point to your fork of Scenic instead of the ``scenic`` package on PyPI.

.. _VerifAI: https://github.com/BerkeleyLearnVerify/VerifAI

MacOS
-----

.. _pythonfcl:

Installing python-fcl on Apple silicon
++++++++++++++++++++++++++++++++++++++

If on an Apple-silicon machine you get an error related to pip being unable to install ``python-fcl``, it can be installed manually using the following steps:

1. Clone the `python-fcl <https://github.com/BerkeleyAutomation/python-fcl>`_ repository.
2. Navigate to the repository.
3. Install dependencies using `Homebrew <https://brew.sh>`__ with the following command: :command:`brew install fcl eigen octomap`
4. Activate your virtual environment if you haven't already.
5. Install the package using pip with the following command: :command:`CPATH=$(brew --prefix)/include:$(brew --prefix)/include/eigen3 LD_LIBRARY_PATH=$(brew --prefix)/lib python -m pip install .`

Windows
-------

.. _wsl:

Using WSL
+++++++++

For greatest ease of installation, we recommend using the `Windows Subsystem for Linux <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_ (WSL, a.k.a. "Bash on Windows") on Windows 10 and newer.

Some WSL users have reported encountering the error ``no display name and no $DISPLAY environmental variable``, but have had success applying the techniques outlined `here <https://github.com/microsoft/WSL/issues/4106#issuecomment-876470388>`_.

It is possible to run Scenic natively on Windows; however, in the past there have been issues with some of Scenic's dependencies either not providing wheels for Windows or requiring manual installation of additional libraries.

Problems building Shapely
+++++++++++++++++++++++++

In the past, the ``shapely`` package did not install properly on Windows.
If you encounter this issue, try installing it manually following the instructions `here <https://github.com/Toblerity/Shapely#built-distributions>`__.
