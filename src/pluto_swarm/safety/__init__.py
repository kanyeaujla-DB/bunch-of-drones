"""Safety layers (architecture doc §7): preflight validation, runtime separation
filter, watchdogs + failsafe state machine, operator kill. P0 has the operator
kill (Ctrl+C) and basic preflight inside swarm/manager.py.
"""
