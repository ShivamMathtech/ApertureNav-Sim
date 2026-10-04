"""Create a portable source ZIP, including compiled dashboard and example reports."""
from pathlib import Path
import zipfile
root=Path(__file__).resolve().parents[1]
out=root.parent/'ApertureNav-Sim-v0.1.0.zip'
excluded={'node_modules','.venv','.venv-ros','.git','__pycache__','build','install','log','.pytest_cache'}
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
    for p in sorted(root.rglob('*')):
        if p.is_file() and not excluded.intersection(p.relative_to(root).parts) and p.suffix not in ('.pyc','.zip'):
            z.write(p,Path(root.name)/p.relative_to(root))
with zipfile.ZipFile(out) as z:
    assert z.testzip() is None
print(out)
print(f'{out.stat().st_size:,} bytes')
