"""Run the existing chapter checks with this interpreter, from any directory."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CHECKS = (
    ('Chapter_4.2_Code', 'test_planning.py'),
    ('Chapter_4.2_Code', 'test_sampling_dynamics.py'),
    ('Chapter_4.2_Code', 'test_simulations.py'),
    ('Chapter_4.3_Code', 'test_trajectory_generation.py'),
)

def main():
    if not __debug__:
        raise SystemExit('Run without -O: the chapter checks use assertions.')
    for folder, name in CHECKS:
        directory = ROOT / 'examples' / 'chapter04' / folder
        print(f'Running {folder}/{name}', flush=True)
        subprocess.run([sys.executable, name], cwd=directory, check=True)
    print('All four Python check programs passed.', flush=True)

if __name__ == '__main__':
    main()
