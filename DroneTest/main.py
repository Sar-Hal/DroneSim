# main.py
# ============================================================
#  DroneSim — Entry point
#  Run:  python main.py
# ============================================================

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

from ursina import *

from config.settings  import WORLD, DRONE_PHYSICS
from src.drone        import Drone
from src.world        import build_world
from src.camera_controller import CameraController
from src.physics      import resolve_tower_collisions, update_obstacles
from src.hud          import HUD


# ── App bootstrap ─────────────────────────────────────────────
app = Ursina()

window.title      = 'DroneSim'
window.borderless = False
window.color      = WORLD["sky_color"]

# Optional: simple distance fog (comment out if performance is tight)
# scene.fog_color   = WORLD["fog_color"]
# scene.fog_density = WORLD["fog_density"]


# ── Build scene ───────────────────────────────────────────────
ground, obstacles = build_world()
drone = Drone()

cam_ctrl = CameraController(drone)
hud      = HUD(drone, cam_ctrl)


def reset_drone_state():
    drone.position = Vec3(0, 6, 0)
    drone.velocity = Vec3(0, 0, 0)
    drone.acceleration = Vec3(0, 0, 0)
    drone.roll = 0
    drone.pitch = 0
    drone.roll_vel = 0
    drone.pitch_vel = 0
    drone.rotation_y = 0


# ── Input ─────────────────────────────────────────────────────
def input(key):
    if key == 'c':
        cam_ctrl.cycle()
    if key == 'r':
        reset_drone_state()


# ── Main loop ─────────────────────────────────────────────────
def update():
    dt = time.dt

    # ── Legacy thrust & attitude controls ─────────────────────
    hover = -drone.gravity
    drone.acceleration = Vec3(0, drone.gravity + hover, 0)

    if held_keys['up arrow']:
        drone.acceleration += drone.forward * drone.move_power
        drone.pitch_vel += 55 * dt
    if held_keys['down arrow']:
        drone.acceleration -= drone.forward * drone.move_power
        drone.pitch_vel -= 55 * dt

    if held_keys['left arrow']:
        drone.acceleration -= drone.right * drone.move_power * 0.8
        drone.roll_vel += 70 * dt
    if held_keys['right arrow']:
        drone.acceleration += drone.right * drone.move_power * 0.8
        drone.roll_vel -= 70 * dt

    if held_keys['space']:
        drone.acceleration.y += drone.thrust_power
    elif held_keys['left shift'] or held_keys['shift']:
        drone.acceleration.y -= drone.thrust_power * 0.72
    else:
        # Auto altitude hold when throttle keys are released.
        drone.acceleration.y += -drone.velocity.y * drone.altitude_hold_damping

    if held_keys['z']:
        drone.rotation_y -= drone.yaw_rate * dt
    if held_keys['x']:
        drone.rotation_y += drone.yaw_rate * dt

    # Return-to-level when no stick input.
    if not held_keys['left arrow'] and not held_keys['right arrow']:
        drone.roll *= max(0.0, 1 - dt * 2.3)
    if not held_keys['up arrow'] and not held_keys['down arrow']:
        drone.pitch *= max(0.0, 1 - dt * 2.3)

    # ── Systems tick ──────────────────────────────────────
    drone.spin_props(dt)
    drone.update_physics(dt)
    resolve_tower_collisions(drone, obstacles)
    update_obstacles(obstacles, dt)
    cam_ctrl.update()
    hud.update()


app.run()