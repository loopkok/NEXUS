"""Build a MuJoCo model from the installed XNero/XHand URDF description.

The URDF remains the source of truth for link geometry, inertias, and frame
transforms. This small converter handles the fixed and revolute joints used by
``xhand_nero_description`` and keeps the model free of duplicated mesh assets.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def _numbers(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default
    values = tuple(float(part) for part in value.split())
    if not all(math.isfinite(part) for part in values):
        raise ValueError(f"URDF contains non-finite vector {value!r}")
    return values


def _rpy_quat(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _fmt(values) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _quat_multiply(left: tuple[float, float, float, float],
                   right: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _axis_angle_quat(axis: tuple[float, ...], angle: float) -> tuple[float, float, float, float]:
    half = angle * 0.5
    sine = math.sin(half)
    return (math.cos(half), axis[0] * sine, axis[1] * sine, axis[2] * sine)


def _mj_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]", "_", value)


def _origin(element: ET.Element | None):
    if element is None:
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    return (
        _numbers(element.get("xyz"), (0.0, 0.0, 0.0)),
        _rpy_quat(_numbers(element.get("rpy"), (0.0, 0.0, 0.0))),
    )


def _mesh_path(uri: str, urdf_path: Path) -> Path:
    prefix = "package://xhand_nero_description/"
    if uri.startswith(prefix):
        path = urdf_path.parent.parent / uri[len(prefix):]
    elif uri.startswith("file://"):
        path = Path(uri[7:])
    else:
        path = Path(uri)
        if not path.is_absolute():
            path = urdf_path.parent / path
    path = path.resolve()
    if path.suffix.lower() == ".dae":
        path = path.with_suffix(".stl")
    if not path.is_file():
        raise FileNotFoundError(f"URDF mesh does not exist: {uri} (resolved to {path})")
    return path


def _rgba(link: ET.Element) -> tuple[float, float, float, float]:
    visual = link.find("visual")
    material = visual.find("material/color") if visual is not None else None
    if material is None:
        name = link.get("name", "")
        if "hand" in name:
            return (0.13, 0.15, 0.18, 1.0)
        if "left_" in name:
            return (0.24, 0.48, 0.72, 1.0)
        if "right_" in name:
            return (0.78, 0.39, 0.22, 1.0)
        return (0.35, 0.38, 0.42, 1.0)
    return _numbers(material.get("rgba"), (0.5, 0.5, 0.5, 1.0))


def _safe_inertia(inertial: ET.Element,
                  rpy: tuple[float, float, float]) -> tuple[float, float, float, float, float, float]:
    row = inertial.find("inertia")
    if row is None:
        return (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0)
    xx = float(row.get("ixx", "0"))
    xy = float(row.get("ixy", "0"))
    xz = float(row.get("ixz", "0"))
    yy = float(row.get("iyy", "0"))
    yz = float(row.get("iyz", "0"))
    zz = float(row.get("izz", "0"))
    # URDF exports for cosmetic hand-cover links sometimes contain tiny,
    # non-positive-definite inertias. Project only those cases to a small
    # positive tensor so MuJoCo can compile the rigid-body model.
    try:
        import numpy as np

        matrix = np.asarray([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]], dtype=float)
        eigenvalues, vectors = np.linalg.eigh(matrix)
        if not np.isfinite(eigenvalues).all():
            raise ValueError("non-finite link inertia")
        if float(eigenvalues.min()) <= 1e-12:
            matrix = vectors @ np.diag(np.maximum(eigenvalues, 1e-10)) @ vectors.T
            xx, xy, xz = matrix[0]
            yy, yz, zz = matrix[1, 1], matrix[1, 2], matrix[2, 2]
    except ImportError:
        smallest_bound = min(xx - abs(xy) - abs(xz),
                             yy - abs(xy) - abs(yz),
                             zz - abs(xz) - abs(yz))
        if smallest_bound <= 1e-12:
            xx, yy, zz = max(xx, 1e-10), max(yy, 1e-10), max(zz, 1e-10)
            xy = xz = yz = 0.0
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    try:
        import numpy as np

        tensor = np.asarray([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]], dtype=float)
        rot = np.asarray(rotation, dtype=float)
        tensor = rot @ tensor @ rot.T
        xx, xy, xz = tensor[0]
        yy, yz, zz = tensor[1, 1], tensor[1, 2], tensor[2, 2]
    except ImportError:
        # NEXUS packages install numpy, but keep this utility importable in a
        # minimal parser environment. The normal ROS runtime takes the path above.
        pass
    return (xx, yy, zz, xy, xz, yz)


def build_mjcf_from_urdf(
    urdf_path: str | Path,
    joint_limits: dict[str, tuple[float, float]] | None = None,
    *,
    joint_zero_offsets: dict[str, float] | None = None,
    timestep: float = 0.002,
) -> str:
    """Convert the XNero/XHand URDF assembly to an in-memory MJCF document.

    ``joint_limits`` may override URDF ranges with the profile's canonical
    hardware ranges. ``joint_zero_offsets`` rotates a child link's zero frame
    about the URDF joint axis while leaving the NEXUS motor-angle coordinate
    unchanged. Nero J2 uses this because its URDF coordinate is shifted by
    -90 degrees from the motor-angle coordinate used by the IK profile.
    """
    path = Path(urdf_path).resolve()
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"expected a URDF <robot>, got <{root.tag}>")
    links = {link.get("name"): link for link in root.findall("link")}
    joints = root.findall("joint")
    children: dict[str, list[ET.Element]] = {}
    child_names: set[str] = set()
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.get('name')} is missing parent/child")
        parent_name, child_name = parent.get("link"), child.get("link")
        if parent_name not in links or child_name not in links:
            raise ValueError(f"joint {joint.get('name')} references an unknown link")
        children.setdefault(parent_name, []).append(joint)
        if child_name in child_names:
            raise ValueError(f"URDF link {child_name!r} has multiple parent joints")
        child_names.add(child_name)
    roots = set(links) - child_names
    if roots != {"world"}:
        raise ValueError(f"expected one URDF world root, found {sorted(roots)}")

    mjcf = ET.Element("mujoco", {"model": "xhand_nero_dual_arm"})
    ET.SubElement(mjcf, "compiler", {
        "angle": "radian", "autolimits": "true", "boundmass": "1e-7",
        "boundinertia": "1e-12", "inertiafromgeom": "false",
    })
    ET.SubElement(mjcf, "option", {
        "timestep": f"{timestep:.9g}", "integrator": "implicitfast",
        "gravity": "0 0 -9.81", "cone": "elliptic",
    })
    ET.SubElement(mjcf, "size", {"njmax": "1000", "nconmax": "500"})
    defaults = ET.SubElement(mjcf, "default")
    ET.SubElement(defaults, "joint", {"damping": "0.04", "armature": "0.001"})
    ET.SubElement(defaults, "geom", {
        "contype": "1", "conaffinity": "1", "condim": "4",
        "friction": "0.9 0.02 0.002", "solref": "0.006 1",
        "solimp": "0.95 0.99 0.001",
    })

    asset = ET.SubElement(mjcf, "asset")
    ET.SubElement(asset, "texture", {
        "name": "sky", "type": "skybox", "builtin": "gradient",
        "rgb1": "0.28 0.37 0.52", "rgb2": "0.02 0.025 0.04",
        "width": "512", "height": "3072",
    })
    ET.SubElement(asset, "material", {
        "name": "ground_material", "rgba": "0.20 0.22 0.25 1",
        "texrepeat": "6 6", "reflectance": "0.08",
    })
    mesh_ids: dict[tuple[str, tuple[float, float, float]], str] = {}
    for link in links.values():
        collision = link.find("collision")
        visual = link.find("visual")
        geometry = (collision if collision is not None else visual)
        if geometry is None:
            continue
        mesh = geometry.find("geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            raise ValueError(f"link {link.get('name')} has unsupported non-mesh geometry")
        mesh_path = _mesh_path(mesh.get("filename"), path)
        scale = _numbers(mesh.get("scale"), (1.0, 1.0, 1.0))
        key = (str(mesh_path), scale)
        if key not in mesh_ids:
            mesh_id = f"mesh_{len(mesh_ids):03d}_{_mj_name(link.get('name', 'link'))}"
            mesh_ids[key] = mesh_id
            ET.SubElement(asset, "mesh", {
                "name": mesh_id, "file": mesh_path.as_posix(), "scale": _fmt(scale),
            })

    worldbody = ET.SubElement(mjcf, "worldbody")
    ET.SubElement(worldbody, "light", {
        "name": "key_light", "pos": "0 -1.5 2.5", "dir": "0 0.4 -1",
        "diffuse": "0.9 0.9 0.9", "specular": "0.2 0.2 0.2",
    })
    ET.SubElement(worldbody, "geom", {
        "name": "ground", "type": "plane", "pos": "0 0 0",
        "size": "2 2 0.1", "material": "ground_material",
    })
    table = ET.SubElement(worldbody, "body", {"name": "task_table", "pos": "0.75 0 0.30"})
    ET.SubElement(table, "geom", {
        "name": "table_top", "type": "box", "pos": "0 0 0.025",
        "size": "0.48 0.66 0.025", "rgba": "0.32 0.26 0.20 1",
        "friction": "1.1 0.03 0.003",
    })
    cube = ET.SubElement(worldbody, "body", {"name": "pick_cube", "pos": "0.65 0 0.39"})
    ET.SubElement(cube, "freejoint", {"name": "pick_cube_free"})
    ET.SubElement(cube, "inertial", {
        "pos": "0 0 0", "mass": "0.08", "diaginertia": "0.000133333 0.000133333 0.000133333",
    })
    ET.SubElement(cube, "geom", {
        "name": "pick_cube_geom", "type": "box", "size": "0.025 0.025 0.025",
        "rgba": "0.92 0.70 0.13 1",
    })

    moving_joints: list[tuple[str, tuple[float, float], float, str]] = []
    visited: set[str] = set()

    def add_link(parent_body: ET.Element, link_name: str,
                 joint: ET.Element | None = None) -> None:
        if link_name in visited:
            raise ValueError(f"URDF contains a cycle at link {link_name!r}")
        visited.add(link_name)
        if joint is None:
            body = ET.SubElement(parent_body, "body", {"name": _mj_name(link_name)})
        else:
            position, quat = _origin(joint.find("origin"))
            joint_type = joint.get("type")
            joint_name = joint.get("name", "")
            offset = float((joint_zero_offsets or {}).get(joint_name, 0.0))
            if not math.isfinite(offset):
                raise ValueError(f"invalid zero offset for joint {joint_name!r}: {offset}")
            if abs(offset) > 1e-12:
                axis_row = joint.find("axis")
                axis = _numbers(axis_row.get("xyz") if axis_row is not None else None,
                                (0.0, 0.0, 1.0))
                norm = math.sqrt(sum(value * value for value in axis))
                if norm <= 1e-12:
                    raise ValueError(f"joint {joint_name!r} has a zero axis")
                axis = tuple(value / norm for value in axis)
                quat = _quat_multiply(quat, _axis_angle_quat(axis, offset))
            body = ET.SubElement(parent_body, "body", {
                "name": _mj_name(link_name), "pos": _fmt(position), "quat": _fmt(quat),
            })
            if joint_type != "fixed":
                if joint_type != "revolute":
                    raise ValueError(f"unsupported joint type {joint_type!r}: {joint.get('name')}")
                limit = joint.find("limit")
                if limit is None:
                    raise ValueError(f"revolute joint {joint.get('name')} has no URDF limits")
                low, high = _numbers(
                    f"{limit.get('lower')} {limit.get('upper')}", (0.0, 0.0))
                if joint_limits and joint_name in joint_limits:
                    low, high = joint_limits[joint_name]
                if not math.isfinite(low) or not math.isfinite(high) or low >= high:
                    raise ValueError(f"invalid limits for joint {joint_name!r}: {(low, high)}")
                axis_row = joint.find("axis")
                axis = _numbers(axis_row.get("xyz") if axis_row is not None else None,
                                (0.0, 0.0, 1.0))
                axis_norm = math.sqrt(sum(value * value for value in axis))
                if axis_norm <= 1e-12:
                    raise ValueError(f"joint {joint_name!r} has a zero axis")
                axis = tuple(value / axis_norm for value in axis)
                hand_joint = "hand_" in joint_name
                ET.SubElement(body, "joint", {
                    "name": _mj_name(joint_name), "type": "hinge", "axis": _fmt(axis),
                    "limited": "true", "range": _fmt((low, high)),
                    "damping": "0.025" if hand_joint else "0.5",
                    "armature": "0.0002" if hand_joint else "0.003",
                })
                effort = float(limit.get("effort", "1"))
                moving_joints.append((joint_name, (low, high), effort, "hand" if hand_joint else "arm"))

        link = links[link_name]
        inertial = link.find("inertial")
        if inertial is not None:
            inertial_origin = inertial.find("origin")
            pos, _ = _origin(inertial_origin)
            inertial_rpy = _numbers(inertial_origin.get("rpy") if inertial_origin is not None else None,
                                    (0.0, 0.0, 0.0))
            mass_row = inertial.find("mass")
            mass = float(mass_row.get("value", "0")) if mass_row is not None else 0.0
            if not math.isfinite(mass) or mass <= 0:
                raise ValueError(f"link {link_name!r} has invalid inertial mass {mass}")
            ET.SubElement(body, "inertial", {
                "pos": _fmt(pos), "mass": f"{mass:.12g}",
                "fullinertia": _fmt(_safe_inertia(inertial, inertial_rpy)),
            })

        geometry = link.find("collision")
        if geometry is None:
            geometry = link.find("visual")
        if geometry is not None:
            mesh = geometry.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"link {link_name!r} has unsupported non-mesh geometry")
            mesh_path = _mesh_path(mesh.get("filename", ""), path)
            scale = _numbers(mesh.get("scale"), (1.0, 1.0, 1.0))
            mesh_id = mesh_ids[(str(mesh_path), scale)]
            pos, quat = _origin(geometry.find("origin"))
            ET.SubElement(body, "geom", {
                "name": _mj_name(f"{link_name}_collision"), "type": "mesh",
                "mesh": mesh_id, "pos": _fmt(pos), "quat": _fmt(quat),
                "rgba": _fmt(_rgba(link)),
            })

        for child_joint in children.get(link_name, []):
            child_link = child_joint.find("child").get("link")
            add_link(body, child_link, child_joint)

    for root_joint in children.get("world", []):
        add_link(worldbody, root_joint.find("child").get("link"), root_joint)
    missing = set(links) - visited - {"world"}
    if missing:
        raise ValueError(f"URDF links are disconnected from world: {sorted(missing)}")

    contact = ET.SubElement(mjcf, "contact")
    excluded_pairs: set[tuple[str, str]] = set()
    for joint in joints:
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if parent == "world":
            continue
        pair = tuple(sorted((parent, child)))
        excluded_pairs.add(pair)
    for side in ("left", "right"):
        # Those housings overlap slightly in the source collision meshes at the
        # nominal wrist pose. They are separated by intervening links and are
        # not intended to collide in the physical assembly.
        excluded_pairs.add(tuple(sorted((f"{side}_link5", f"{side}_link7"))))
        excluded_pairs.add(tuple(sorted((f"{side}_link5", f"{side}_gripper_flange"))))
    for body1, body2 in sorted(excluded_pairs):
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})

    actuators = ET.SubElement(mjcf, "actuator")
    for name, limits, effort, group in moving_joints:
        low, high = limits
        ctrl_low, ctrl_high = min(low, high), max(low, high)
        if group == "hand":
            kp, kv = 3.0, 0.08
        else:
            # The URDF's wrist inertias are small.  A 220/25 servo saturates
            # at 100 Nm on the left wrist and enters a sustained limit cycle
            # at common teleop poses, while the mirrored right arm tracks.
            # This lower gain pair settles both sides within 0.5 s at the
            # configured 2 ms physics step.
            kp, kv = 100.0, 10.0
        max_effort = max(0.01, abs(effort))
        ET.SubElement(actuators, "position", {
            "name": _mj_name(f"act_{name}"), "joint": _mj_name(name),
            "kp": f"{kp:.6g}", "kv": f"{kv:.6g}",
            "ctrllimited": "true", "ctrlrange": _fmt((ctrl_low, ctrl_high)),
            "forcelimited": "true", "forcerange": _fmt((-max_effort, max_effort)),
        })
    ET.SubElement(mjcf, "keyframe")
    return ET.tostring(mjcf, encoding="unicode")
