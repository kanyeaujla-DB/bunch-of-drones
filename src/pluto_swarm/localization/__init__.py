"""Localization: camera -> detect -> pose -> associate -> estimate -> health.

OUTPUT CONTRACT: one core.types.DroneState per drone per update, arena frame,
t = capture time, health = OK / DEGRADED / LOST. Control only ever reads that.
"""
