"""The Scenic compiler and associated support code."""

import pathlib as _pathlib
import subprocess as _subprocess
import sys as _sys

_syntaxDir = _pathlib.Path(__file__).parent
_projectRootDir = _syntaxDir.parent.parent.parent
_grammarPath = _syntaxDir / "scenic.gram"
_parserPath = _syntaxDir / "parser.py"


def buildParser():
    try:
        import pegen
    except ModuleNotFoundError:
        raise RuntimeError(
            f'the "pegen" package is required to build the Scenic parser'
        ) from None
    result = _subprocess.run(
        [
            _sys.executable or "python",
            "-m",
            "pegen",
            str(_grammarPath),
            "-o",
            str(_parserPath),
        ],
        cwd=_projectRootDir,
        capture_output=True,
        text=True,
    )
    return result


if not _parserPath.exists():
    _result = buildParser()
    _retcode = _result.returncode
    if _retcode != 0:
        print(f"STDERR FROM PARSER GENERATOR:\n{_result.stderr}", file=_sys.stderr)
        raise RuntimeError(
            f"Failed to generate the Scenic parser (return code {_retcode})"
        )
