#!/usr/bin/env python3
"""Apply the workspace's Jazzy compatibility patches to the pinned fork revisions."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
BASES = {
    'rmf_demos': '91f7c07853b0e0fd8694bc45cb9f91ad08a2a797',
    'clearpath_simulator': '70235ea574cd1efdc74327904a25b9ec3f57e257',
}


def main():
    pending = []
    for name, base in BASES.items():
        directory = ROOT / 'src' / name
        patch = ROOT / 'scripts' / 'submodule_patches' / (name + '-jazzy.patch')
        head = subprocess.check_output(['git', '-C', str(directory), 'rev-parse', 'HEAD'],
                                       text=True).strip()
        if head != base:
            raise SystemExit(f'{name}: expected pinned revision {base}, found {head}; review the patch before updating.')
        command = ['git', '-C', str(directory), 'apply']
        if subprocess.run(command + ['--reverse', '--check', str(patch)],
                          capture_output=True).returncode == 0:
            print(f'{name}: Jazzy compatibility patch already applied')
            continue
        subprocess.run(command + ['--check', str(patch)], check=True)
        pending.append((name, command, patch))
    # Check every patch first, so a conflict cannot leave a partially patched pair.
    for name, command, patch in pending:
        subprocess.run(command + [str(patch)], check=True)
        print(f'{name}: applied Jazzy compatibility patch')


if __name__ == '__main__':
    main()
