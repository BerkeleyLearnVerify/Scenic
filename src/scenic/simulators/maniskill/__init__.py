"""
Interface to the ManiSkill robotics simulator.
"""

# Check if mani_skill is installed and import ManiSkillSimulator only if it is
mani_skill = None
try:
    import mani_skill
except ImportError:
    pass
if mani_skill:
    from .simulator import ManiSkillSimulator
del mani_skill
