"""Interface to the LGSVL driving simulator.

The LGSVL Simulator interface was deprecated in Scenic 3. To continue to use the interface, please use Scenic 2.
"""

# Only import LGSVLSimulator if the lgsvl package is installed; otherwise the
# import would raise an exception.
lgsvl = None
try:
    import lgsvl
except ImportError:
    pass
if lgsvl:
    from .simulator import LGSVLSimulator
del lgsvl
