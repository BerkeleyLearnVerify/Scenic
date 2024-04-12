############################################
Getting Started with VerifAI
############################################

VerifAI requires **Python 3.8** or newer.
Run :command:`python --version` to make sure you have a new enough version; if not, you can install one from the `Python website <https://www.python.org/downloads/>`_ or using `pyenv <https://github.com/pyenv/pyenv>`_ (e.g. running :command:`pyenv install 3.11`).
If the version of Python you want to use is called something different than just ``python`` on your system, e.g. ``python3.11``, use that name in place of ``python`` throughout the following instructions.

There are two ways to install VerifAI:

* from our repository, which has the very latest features but may not be stable. The repository also contains example scripts such as those used in the tutorial.

* from the Python Package Index (PyPI), which will get you the latest official release of VerifAI but will not include example code, etc.

If this is your first time using VerifAI, we suggest installing from the repository so that you can try out the examples.

Once you've decided which method you want to use, follow the instructions below, which should work on macOS and Linux (on Windows, we recommend using the Windows Subsystem for Linux).

First, activate the `virtual environment <https://docs.python.org/3/tutorial/venv.html>`_ in which you want to install VerifAI.
To create and activate a new virtual environment called :file:`venv`, you can run the following commands:

.. code-block:: text

	python -m venv venv
	source venv/bin/activate

Once your virtual environment is activated, make sure your :command:`pip` tool is up-to-date:

.. code-block:: text

	python -m pip install --upgrade pip

Now you can install VerifAI either from the repository or from PyPI:

.. tabs::

	.. tab:: Repository

		The following commands will clone the `VerifAI repository <https://github.com/BerkeleyLearnVerify/VerifAI>`_ into a folder called :file:`VerifAI` and install VerifAI from there.
		It is an "editable install", so if you later update the repository with :command:`git pull` or make changes to the code yourself, you won't need to reinstall VerifAI.

		.. code-block:: text

			git clone https://github.com/BerkeleyLearnVerify/VerifAI
			cd VerifAI
			python -m pip install -e .

	.. tab:: PyPI

		The following command will install the latest full release of Scenic from `PyPI <https://pypi.org/project/verifai/>`_:

		.. code-block:: text

			python -m pip install verifai

		Note that this command skips experimental alpha and beta releases, preferring stable versions.
		If you want to get the very latest version available on PyPI (which may still be behind the repository), run:

		.. code-block:: text

			python -m pip install --pre verifai

		You can also install specific versions with a command like:

		.. code-block:: text

			python -m pip install verifai==2.1.0b1

Some features of VerifAI require additional packages: the tool will prompt you if they are needed but not installed.

.. note::

	In the past, the ``GPy`` package did not always install correctly through the automated process. If necessary, you can build it from source as follows::

		git clone https://github.com/SheffieldML/GPy
		find GPy -name '*.pyx' -exec cython {} \
		pip install GPy/
