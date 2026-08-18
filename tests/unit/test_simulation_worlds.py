from pathlib import Path
from xml.etree import ElementTree


def test_plane_world_has_dense_ground_features_for_custom_tracking() -> None:
    world_path = Path("simulation/worlds/vulture_x_plane.sdf")
    root = ElementTree.parse(world_path).getroot()

    feature_field = root.find(".//model[@name='tracking_feature_field']")
    assert feature_field is not None
    grass_visuals = [
        visual
        for visual in feature_field.findall(".//visual")
        if visual.attrib.get("name", "").startswith("grass_blade_")
    ]

    assert len(grass_visuals) >= 24
    assert all(visual.find("material") is not None for visual in grass_visuals)


def test_plane_world_name_matches_ui_target_profile() -> None:
    world_path = Path("simulation/worlds/vulture_x_plane.sdf")
    root = ElementTree.parse(world_path).getroot()
    world = root.find("world")

    assert world is not None
    assert world.attrib["name"] == "vulture_x_plane"
