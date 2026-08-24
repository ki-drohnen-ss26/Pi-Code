"""
conftest.py
===========
Makes the flat module layout (camera.py, config.py, drone.py, ...) importable
from the tests under tests/, regardless of where pytest is invoked from. Its mere
presence at the project root also tells pytest to add this directory to sys.path.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
