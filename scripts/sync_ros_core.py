"""Refresh the ROS distribution copy of the canonical shared algorithms."""
from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1]
shutil.copytree(root/'aperturenav',root/'drone_ws/src/aperturenav_core/aperturenav',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy(root/'config/default.yaml',root/'drone_ws/src/drone_bringup/config/default.yaml')
shutil.copy(root/'config/default.yaml',root/'drone_ws/src/aperturenav_core/config/default.yaml')
shutil.copy(root/'tests/test_core.py',root/'drone_ws/src/aperturenav_core/tests/test_core.py')
print('Shared core synchronized')
