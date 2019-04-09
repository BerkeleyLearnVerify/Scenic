
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
    pass

class RuntimeParseError(ParseError):
	pass

class InvalidScenarioError(Exception):
    pass
