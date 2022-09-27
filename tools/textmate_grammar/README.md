# TextMate Grammar for Scenic

This folder contains a [TextMate-format](https://macromates.com/manual/en/language_grammars) grammar for Scenic, suitable for use with Sublime Text, Atom, and VS Code. The grammar is based on the [MagicPython grammar](https://github.com/MagicStack/MagicPython) by MagicStack Inc., which is available under the MIT license (see `LICENSE` in this directory).

## Usage

The main grammar file is `Scenic.yaml`.
To prepare a `.tmLanguage` file for Sublime Text, you can use the `syntaxdev` tool as follows:

```
npm install syntaxdev@0.0.16
./node_modules/.bin/syntaxdev build-plist --in Scenic.yaml --out Scenic.tmLanguage
```

See the installation instructions for [MagicPython](https://github.com/MagicStack/MagicPython) for additional information.

## Testing

The `polychrome.scenic` file contains almost all valid Scenic syntax: you can use it to test out different color schemes (which are set by your editor, not by this grammar) and any changes you make to the grammar.