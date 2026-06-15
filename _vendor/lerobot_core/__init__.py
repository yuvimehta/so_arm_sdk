"""Self-contained subset of the official ``lerobot`` package.

Only the modules required by :mod:`lerobot_sdk` (the SO-100/SO-101 follower
robot and the placo-based kinematics solver, plus their transitive
dependencies) are vendored here. All original ``from lerobot.*`` imports have
been rewritten to relative imports within this package.
"""
