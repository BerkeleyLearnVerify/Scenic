
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
    joinedArgs = ', '.join(str(a) for a in args)
    return f'({joinedArgs})'

class ParseError(Exception):
    pass

class RuntimeParseError(ParseError):
	pass
