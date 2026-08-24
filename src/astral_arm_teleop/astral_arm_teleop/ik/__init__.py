"""Astral dual-arm IK layer (torso ``base_link``).

Layout::

  ik/
    base.py          — IKSolverBase protocol
    robot_params.py  — Modified DH parameters
    analytic.py      — Nero-port IKSolver / AstralParams (closed-form DH)
    urdf_solver.py   — URDFNumericalIKSolver (Pinocchio LM)
    factory.py       — AstralIKBridge + make_ik_solver

Use ``make_ik_solver(solver_type)`` from teleop; do not import solvers ad-hoc.
"""

from astral_arm_teleop.ik.factory import (
    AstralIKBridge,
    default_astral_urdf_path,
    default_urdf_path,
    make_ik_solver,
    make_single_arm_ik,
)

__all__ = [
    "AstralIKBridge",
    "default_astral_urdf_path",
    "default_urdf_path",
    "make_ik_solver",
    "make_single_arm_ik",
]
