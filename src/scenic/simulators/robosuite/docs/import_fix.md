## Fix Documentation: Preventing RoboSuite Import During Compilation

**Problem:** RoboSuite was importing when running `scenic` without `-S` flag, causing warnings even when only compiling scenarios.

**Root Cause:** `simulator.py` had RoboSuite imports at module level, executing when `model.scenic` imported from it.

**Solution:** Moved all RoboSuite imports inside `RobosuiteSimulator.__init__()` to defer loading until simulator creation.

### Changes Made:

1. **Removed module-level imports** in `simulator.py`:
   - Deleted the try/except block importing RoboSuite
   - Removed global `_ROBOSUITE_AVAILABLE` check

2. **Deferred imports to runtime** in `RobosuiteSimulator.__init__()`:
   ```python
   def __init__(self, ...):
       # Import RoboSuite only when creating simulator
       try:
           global mujoco, suite
           import mujoco
           import robosuite as suite
       except ImportError as e:
           raise RuntimeError(f"Unable to import RoboSuite: {e}")
   ```

3. **Kept deferred class definitions** (`_get_arena_class()`, `_get_object_class()`, `_get_env_class()`) which are called in `RobosuiteSimulation.__init__()`

4. **Added fallback in model.scenic** (already done):
   - Try/except handles case when RoboSuite isn't installed at all

**Result:** RoboSuite only imports when running with `-S` flag, eliminating warnings during scenario compilation.
