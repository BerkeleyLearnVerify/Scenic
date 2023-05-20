"""Converter from Scenic 2.x to 3.0.

This module is designed to be invoked from the command line.
Run the following command to see the available options:

.. code-block:: console

    $ python -m scenic.syntax.scenic2to3 -h
"""

import argparse
import os.path
import tokenize

import scenic
import scenic.syntax.translator as translator 

class Scenic2to3:
    def __init__(self, path, params, model):
        self.path = path
        self.params = params
        self.model = model
        self.splices = []
        self.monitorName = None
        self.monitorLevel = None
        self.indentLevel = 0
        self.logicalLine = 1

    def dump(self, outPath):
        with open(self.path, 'rb') as stream:
            translator.compileTopLevelStream(stream, self.path, self.params, self.model,
                                             self.path, dumpScenic3=self)
        splices = sorted(self.splices, key=lambda splice: splice[:2])

        with open(self.path, 'rb') as stream:
            encoding = tokenize.detect_encoding(stream.readline)[0]
        with open(self.path, 'r', encoding=encoding) as inStream, \
             open(outPath, 'w', encoding=encoding) as outStream:
            curRow = 0
            offset = 0
            pieces = []
            for row, col, text, erased in splices:
                noNewline = False
                while curRow < row:
                    outStream.write(''.join(pieces))
                    line = inStream.readline()
                    if line and line[-1] != '\n' and curRow+1 < row:
                        # file ends abruptly without newline; add one since
                        # we have another splice coming to go at the end
                        noNewline = True
                        line += '\n'
                    pieces = [line]
                    curRow += 1
                    offset = 0
                piece = pieces.pop()
                spot = col - offset
                pieces.append(piece[:spot])
                if noNewline and text[-1] == '\n':
                    text = text[:-1]
                pieces.append(text)
                pieces.append(piece[spot+erased:])
                offset = col + erased
            outStream.write(''.join(pieces))
            for line in inStream:
                outStream.write(line)

    def addSpliceBefore(self, token, text, row=None, newline=False):
        trow, col = token[2]
        if row is None:
            row = trow
        if newline:
            text += '\n' + ' '*col
        splice = (row, col, text, 0)
        self.splices.append(splice)

    def addSpliceReplacing(self, fromToken, toToken, text):
        srow, scol = fromToken[2]
        erow, ecol = toToken[3]
        assert srow == erow and scol <= ecol, (fromToken, toToken)
        splice = (srow, scol, text, ecol - scol)
        self.splices.append(splice)

    def recordInstanceCreation(self, token):
        # Add 'new' before instance creations.
        self.addSpliceBefore(token, 'new ')

    def recordSpecifier(self, token, nextToken=None):
        if nextToken:
            twoWords = (token.string, nextToken.string)
            if twoWords == ('offset', 'by'):
                # Replace 'offset by' specifiers with 'at ego offset by'.
                self.addSpliceBefore(token, 'at ego ')
            elif twoWords == ('with', 'heading'):
                # Replace 'with heading' specifiers with 'facing'.
                self.addSpliceReplacing(token, nextToken, 'facing')

    def recordMonitor(self, name, token):
        # Add empty parameter lists to monitor definitions, and prepare to add
        # a 'require monitor' statement after the definition.
        self.addSpliceBefore(token, '()')
        self.monitorName = name
        self.monitorLevel = self.indentLevel

    def recordLogicalLine(self, token):
        self.logicalLine = token.start[0]+1

    def recordIndentation(self, token):
        if token.exact_type == tokenize.INDENT:
            self.indentLevel += 1
        elif token.exact_type == tokenize.DEDENT:
            self.indentLevel -= 1
            # Add a 'require monitor' statement if we just ended a monitor definition
            if self.indentLevel == self.monitorLevel:
                text = f'require monitor {self.monitorName}()'
                self.addSpliceBefore(token, text, row=self.logicalLine, newline=True)
                self.monitorName = None
                self.monitorLevel = None
        else:
            raise AssertionError(f'unexpected token {token}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='python -m scenic.syntax.scenic2to3',
        description=('Convert a Scenic 2.x program to one that runs in Scenic 3.0 in '
                     '2D compatibility mode.'),
        epilog=('N.B. Parsing a Scenic 2.x program requires importing it as a module; '
                'if your program cannot be imported without specifying a world model '
                'or certain global parameters, use the corresponding options above.'),
    )
    parser.add_argument('-f', '--force', help='overwrite output file if it exists',
                        action='store_true')
    parser.add_argument('-p', '--param', help='override a global parameter',
                        nargs=2, default=[], action='append', metavar=('PARAM', 'VALUE'))
    parser.add_argument('-m', '--model', help='specify a Scenic world model', default=None)
    parser.add_argument('input', help='a Scenic file to convert')
    parser.add_argument('output', help='filename for converted program')

    args = parser.parse_args()
    if os.path.exists(args.output):
        if not args.force:
            raise RuntimeError('output file already exists (use -f to overwrite)')
        if os.path.samefile(args.input, args.output):
            raise RuntimeError('cannot use the same file for both input and output')
    params = {}
    for name, value in args.param:
        # Convert params to ints or floats if possible
        try:
            value = int(value)
        except ValueError:
            try:
                value = float(value)
            except ValueError:
                pass
        params[name] = value

    converter = Scenic2to3(args.input, params, args.model)
    converter.dump(args.output)
