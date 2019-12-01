
import pytest

from scenic.syntax.translator import TokenParseError
from tests.utils import compileScenic, sampleEgoFrom

templates = [
'''
ego = Object with height 2, {continuation}
{indent}with width 3, {continuation}
{indent}at 10@20
''',
'''
ego = Object with height 2, {continuation}
{indent}with width 3, at 10@20
''',
'''
ego = Object with height 2, {continuation}
{indent}#with width 2,{continuation}
{indent}with width 3
''',
'''
ego = Object with height 2, {continuation}
# with width 2,{continuation}
# blah {continuation}
{indent}with width 3
'''
]

@pytest.mark.parametrize('template', templates)
@pytest.mark.parametrize('continuation', ('', '\\', '# comment'))
@pytest.mark.parametrize('indent', ('  ', '    ', '             '))
@pytest.mark.parametrize('gap', ('', '\n', '# comment\n', '\n# comment\n'))
def test_specifier_layout(template, continuation, indent, gap):
    """Test legal specifier layouts, with and without line continuations."""
    preamble = template.format(continuation=continuation, indent=indent)
    program = preamble + gap + 'Object at 20@20'
    print('TESTING PROGRAM:', program)
    compileScenic(program)

def test_dangling_specifier_list():
    with pytest.raises(TokenParseError):
        compileScenic('ego = Object with width 4,')
    with pytest.raises(TokenParseError):
        compileScenic('ego = Object with width 4,   # comment')

def test_incipit_as_name():
    """Incipits of operators are not keywords and can be used as names.

    Here we try 'distance' from 'distance from X' and 'offset' from 'X offset by Y'.
    """
    for name in ('distance', 'offset'):
        ego = sampleEgoFrom(f'{name} = 4\n' f'ego = Object at {name} @ 0')
        assert tuple(ego.position) == (4, 0)
