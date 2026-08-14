"""
spawn_utils.py — helper de spawn de cajas estáticas en Gazebo, compartido por
las misiones de test. Extraído del patrón usado en mission_nav_test.py.
"""

import subprocess


def spawn_static_box(world: str, name: str, x: float, y: float, z: float,
                      sx: float = 1.0, sy: float = 1.0, sz: float = 2.0,
                      color=(0.8, 0.2, 0.2)) -> bool:
    """Spawn a static box in Ignition Gazebo via ros_gz_sim. Returns True on success."""
    r, g, b = color
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    result = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", world,
         "-string", sdf,
         "-x", str(x), "-y", str(y), "-z", str(z)],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode == 0:
        print(f"  [SPAWN] '{name}' aparecido en ({x}, {y}, {z})")
        return True
    print(f"  [SPAWN ERROR] {result.stderr.strip()}")
    return False
