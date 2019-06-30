
### Utilities

def cached(oldMethod):
    """Decorator for making a method with no arguments cache its result"""
    storageName = f'_cached_{oldMethod.__name__}'
    def newMethod(self):
        if not hasattr(self, storageName):
            setattr(self, storageName, oldMethod(self))
        return getattr(self, storageName)
    return newMethod

def argsToString(args):
    names = (f'{a[0]}={a[1]}' if isinstance(a, tuple) else str(a) for a in args)
    joinedArgs = ', '.join(names)
    return f'({joinedArgs})'

class ParseError(Exception):
    """An error produced by attempting to parse an invalid Scenic program."""
    pass

class RuntimeParseError(ParseError):
    """A Scenic parse error generated during execution of the translated Python."""
    pass

class InvalidScenarioError(Exception):
    """Error raised for syntactically-valid but otherwise problematic Scenic programs."""
    pass

class InconsistentScenarioError(InvalidScenarioError):
    """Error for scenarios with inconsistent requirements."""
    def __init__(self, line, message):
        self.lineno = line
        super().__init__('Inconsistent requirement on line ' + str(line) + ': ' + message)
