# Adding realistic assets (textured meshes, PBR, Fuel)

## What is already in the repo
`mobile_robot_gazebo/models/` holds 7 textured props imported from <https://github.com/osrf/gazebo_models> (CC BY 3.0, see
`models/ATTRIBUTION.md`): cafe table, cardboard box, EUR-pallet, trash bin, storage tote, traffic cone, construction barrel.
`house_world.sdf` places them with `<include><uri>model://cafe_table</uri>...`. The launch adds the folder to `GZ_SIM_RESOURCE_PATH`.

Import more (or re-create these):
```bash
git clone --depth 1 https://github.com/osrf/gazebo_models /tmp/gazebo_models
pip install --break-system-packages trimesh pycollada numpy
python3 mobile_robot_gazebo/tools/import_gazebo_models.py /tmp/gazebo_models   # edit SPECS in the script to add a model
python3 mobile_robot_gazebo/tools/make_house_world.py                          # regenerate the world, then place it with prop(...)
```
The importer fixes what breaks old models in Gazebo Harmonic: mixed units (inch / mm / m), bare texture file names, Gazebo-classic
material scripts, and mesh collisions. Sizes are set to real-world dimensions in `SPECS`.

## Layout for your own CAD / Blender asset
```
mobile_robot_gazebo/models/my_obstacle/
├── model.config
├── model.sdf
├── meshes/my_obstacle.dae          <- textured COLLADA (export from Blender with "Copy textures"; unit = metre)
└── materials/textures/             <- albedo.png, normal.png, roughness.png, metalness.png
```
Rules that keep the simulation fast and stable:
1. **Visual = the detailed `.dae`. Collision = box / cylinder / sphere only.** Never point `<collision>` at a mesh.
2. Unit = metre, origin at the floor, +z up. Use `<scale>` only to correct a source file.
3. Dynamic models need `<mass>` and `<inertia>` (the tests fail otherwise).
4. Reference texture files relative to the `.dae` (`../materials/textures/x.png`).

PBR material (gz-sim with ogre2 renders it; use it on a visual instead of relying on the `.dae` material):
```xml
<visual name="visual">
  <geometry><mesh><uri>model://my_obstacle/meshes/my_obstacle.dae</uri></mesh></geometry>
  <material>
    <pbr><metal>
      <albedo_map>model://my_obstacle/materials/textures/albedo.png</albedo_map>
      <normal_map>model://my_obstacle/materials/textures/normal.png</normal_map>
      <roughness_map>model://my_obstacle/materials/textures/roughness.png</roughness_map>
      <metalness_map>model://my_obstacle/materials/textures/metalness.png</metalness_map>
    </metal></pbr>
  </material>
</visual>
```
Then add `prop("obstacle_1", "my_obstacle", x, y)` in `tools/make_house_world.py` (and the name to the tests happens automatically:
every folder in `models/` is audited).

## Gazebo Fuel (online catalogue)
Browse <https://app.gazebosim.org/fuel/models>. A Fuel model is used by URL, e.g.
`<include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bookshelf</uri></include>`; it downloads on first run (internet
needed). For reproducible, offline CI builds, download the model once and vendor it under `models/` like the imported ones.
Check each model's licence before you publish it.

## Not verified here
The models were scaled and checked with mesh tools and unit tests, not rendered in Gazebo. Open `house_world.sdf` and look
for floating or sunken props; the sizes are in `models/ATTRIBUTION.md` and the pose is in `make_house_world.py`.
