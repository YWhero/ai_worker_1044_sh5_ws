#!/usr/bin/env python3
"""Create the ShelfE visual used by the manipulation test scene.

The AWS ShelfE visual combines the rack and all cargo in one Collada mesh.
This script removes only the cargo on the lowest shelf so those props can be
replaced by independent dynamic SDF models.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": COLLADA_NS}
POSITION_SOURCE_ID = "aws_robomaker_warehouse_ShelfE_01_visual-POSITION"

# Cargo materials are shared across shelf levels.  Their lowest-shelf pieces
# occupy disjoint Z bands, so the thresholds remove complete box components
# without touching the upper shelf cargo.
LOWEST_SHELF_MAX_Z_CM = {
    "Material #946230": 130.0,
    "Material #945824": 100.0,
    "Material #945816": 110.0,
}


def remove_lowest_shelf_cargo(source: Path, destination: Path) -> None:
    ET.register_namespace("", COLLADA_NS)
    tree = ET.parse(source)
    root = tree.getroot()

    position_source = root.find(
        f".//c:source[@id='{POSITION_SOURCE_ID}']/c:float_array", NS
    )
    if position_source is None or not position_source.text:
        raise RuntimeError("ShelfE position array was not found")

    values = [float(value) for value in position_source.text.split()]
    positions = [tuple(values[index:index + 3]) for index in range(0, len(values), 3)]

    removed_triangles = {}
    for triangles in root.findall(".//c:triangles", NS):
        material = triangles.get("material")
        threshold = LOWEST_SHELF_MAX_Z_CM.get(material)
        if threshold is None:
            continue

        inputs = triangles.findall("c:input", NS)
        stride = max(int(input_element.get("offset", "0")) for input_element in inputs) + 1
        vertex_offset = next(
            int(input_element.get("offset", "0"))
            for input_element in inputs
            if input_element.get("semantic") == "VERTEX"
        )
        indices = triangles.find("c:p", NS)
        if indices is None or not indices.text:
            continue

        data = [int(value) for value in indices.text.split()]
        face_width = stride * 3
        kept_faces = []
        removed_count = 0
        for start in range(0, len(data), face_width):
            face = data[start:start + face_width]
            vertex_indices = [face[vertex_offset + stride * corner] for corner in range(3)]
            if max(positions[index][2] for index in vertex_indices) < threshold:
                removed_count += 1
            else:
                kept_faces.extend(face)

        triangles.set("count", str(len(kept_faces) // face_width))
        indices.text = " " + " ".join(str(value) for value in kept_faces)
        removed_triangles[material] = removed_count

    expected = {
        "Material #946230": 14,
        "Material #945824": 24,
        "Material #945816": 72,
    }
    if removed_triangles != expected:
        raise RuntimeError(
            f"Unexpected ShelfE mesh topology: removed {removed_triangles}, expected {expected}"
        )

    tree.write(destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    package_dir = Path(__file__).resolve().parents[1]
    mesh_dir = package_dir / (
        "worlds/aws_robomaker_small_warehouse/models/"
        "aws_robomaker_warehouse_ShelfE_01/meshes"
    )
    remove_lowest_shelf_cargo(
        mesh_dir / "aws_robomaker_warehouse_ShelfE_01_visual.DAE",
        mesh_dir / "aws_robomaker_warehouse_ShelfE_01_manipulation_visual.DAE",
    )
