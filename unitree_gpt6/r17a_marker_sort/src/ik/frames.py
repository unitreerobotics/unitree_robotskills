"""World frame: centre of the underside of the R1-7A base plate, axes parallel to base_link."""

import xml.etree.ElementTree as ET


REFERENCE_FRAME = "world"
BASE_LINK = "base_link"
# Bottom plate underside is at base_link Z=-26 mm. Axes stay parallel.
BASE_IN_WORLD_M = (0.0, 0.0, 0.026)


def add_world_frame(root: ET.Element):
    """Wrap the original SDK robot without changing any motor joint."""
    if root.find(f"link[@name='{REFERENCE_FRAME}']") is not None:
        raise ValueError("SDK robot already defines world; check its installation transform")
    if root.find(f"link[@name='{BASE_LINK}']") is None:
        raise ValueError("SDK robot has no base_link")
    ET.SubElement(root, "link", name=REFERENCE_FRAME)
    joint = ET.SubElement(root, "joint", name="world_to_base_link", type="fixed")
    ET.SubElement(joint, "parent", link=REFERENCE_FRAME)
    ET.SubElement(joint, "child", link=BASE_LINK)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str, BASE_IN_WORLD_M)), rpy="0 0 0")
