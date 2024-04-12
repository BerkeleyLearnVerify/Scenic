
import functools

class frozendict(dict):
    """An immutable dictionary. Hashable if its values are."""

    def _immutableError(self, *args, **kwargs):
        raise TypeError('frozendict is immutable')

    __setitem__ = _immutableError
    __delitem__ = _immutableError
    clear = _immutableError
    pop = _immutableError
    popitem = _immutableError
    setdefault = _immutableError
    update = _immutableError

    @functools.cached_property
    def _hash(self):
        return hash(tuple(self.items()))

    def __hash__(self):
        return self._hash
