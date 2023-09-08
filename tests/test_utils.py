"""Test internal utilities used by the test suite."""

from tests.utils import areEquivalent

# Pre- and post-pickling equivalence checks


def test_equivalence_slots():
    """Check that `areEquivalent` handles __slots__ correctly."""

    def check(x, y, addAttrC=False):
        x.a = 4
        y.a = 4
        assert areEquivalent(x, y)
        y.b = 12
        assert not areEquivalent(x, y)
        x.b = 12
        assert areEquivalent(x, y)
        x.b = 13
        assert not areEquivalent(x, y)
        if addAttrC:
            y.b = 13
            x.c = 42
            assert not areEquivalent(x, y)
            y.c = 42
            assert areEquivalent(x, y)
            y.c = 43
            assert not areEquivalent(x, y)

    # Slots only
    class Foo:
        __slots__ = ("a", "b")

    check(Foo(), Foo())

    # Slots in superclass, plus dict
    class Bar(Foo):
        pass

    check(Bar(), Bar(), addAttrC=True)

    # Dict in superclass, plus slots
    class Baz:
        pass

    class Qux(Baz):
        __slots__ = ("a", "b")

    check(Qux(), Qux(), addAttrC=True)

    # Slots in superclass, plus slots
    class Snafu(Foo):
        __slots__ = ("c",)

    check(Snafu(), Snafu(), addAttrC=True)
