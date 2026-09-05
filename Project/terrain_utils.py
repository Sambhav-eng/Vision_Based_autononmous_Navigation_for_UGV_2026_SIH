"""
Shared terrain/ground texturing helpers for the UGV simulation.

Both camera.py and simulation.py used to have their own copy-pasted version
of this logic, which meant fixing a bug in one file didn't fix the other.
Import from here instead so there's a single source of truth.

Two separate textures are used, because they serve different jobs:
  - "horizon" texture -> your terrain.jpg landscape photo (sky/mountains/trees).
    This belongs on a distant backdrop wall, viewed once, not tiled.
  - "ground" texture -> a procedurally generated, seamless grass texture.
    This belongs on the floor, where PyBullet tiles it many times under
    the robot's wheels. Tiling a landscape photo there is what caused the
    smeared/repeated-sky look - a real horizon photo is never seamless.
"""

import os
import pybullet as p

MAX_TEXTURE_DIM = 2048  # safe upper bound for GPU texture size / stb_image


# ============================================================
# HORIZON / BACKDROP TEXTURE (your terrain.jpg photo)
# ============================================================

def _get_cached_or_resized_path(image_path):
    """
    PyBullet's texture loader (stb_image) can silently fail on very large
    and/or progressive JPEGs. If the source image is too big, resize it to
    a safe baseline JPEG and cache that next to the original, returning the
    cached path. If Pillow isn't installed, or the image is already small
    enough, the original path is returned unchanged.
    """
    try:
        from PIL import Image
    except ImportError:
        print("[WARNING] Pillow (PIL) not installed - cannot auto-resize "
              "oversized textures. Run: pip install pillow")
        return image_path

    folder = os.path.dirname(image_path)
    base, _ext = os.path.splitext(os.path.basename(image_path))
    cached_path = os.path.join(folder, f"{base}_cached.jpg")

    try:
        with Image.open(image_path) as im:
            width, height = im.size
            is_progressive = getattr(im, "info", {}).get("progression", False)
            needs_resize = width > MAX_TEXTURE_DIM or height > MAX_TEXTURE_DIM

            if os.path.exists(cached_path) and not needs_resize and not is_progressive:
                return image_path  # already safe, no cache needed

            if needs_resize or is_progressive:
                im = im.convert("RGB")
                if needs_resize:
                    im.thumbnail((MAX_TEXTURE_DIM, MAX_TEXTURE_DIM), Image.LANCZOS)
                im.save(cached_path, format="JPEG", quality=90, progressive=False)
                print(f"[INFO] Horizon image was {width}x{height} "
                      f"(too large or progressive) - resized copy saved to: {cached_path}")
                return cached_path

    except Exception as e:
        print(f"[WARNING] Could not inspect/resize horizon image ({e}); "
              "using original file as-is.")

    return image_path


def get_horizon_texture_id(project_dir, sub_path=("Assets", "backgrounds", "terrain.jpg")):
    """Loads your landscape photo (auto-resizing if needed) for use on the backdrop wall only."""
    image_path = os.path.join(project_dir, *sub_path)

    print("Loading horizon texture:", image_path)

    if not os.path.exists(image_path):
        print(f"[WARNING] Horizon texture not found at: {image_path}")
        return None

    safe_path = _get_cached_or_resized_path(image_path)
    texture_id = p.loadTexture(safe_path)

    if texture_id < 0:
        print(f"[WARNING] PyBullet failed to load texture: {safe_path}")
        return None

    return texture_id


def create_backdrop_wall(texture_id, position=(45, 0, 20), half_extents=(0.1, 60, 20)):
    """Distant backdrop wall showing the landscape photo once (not tiled)."""
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(half_extents))
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half_extents))

    background = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=list(position)
    )

    if texture_id is not None:
        p.changeVisualShape(background, -1, textureUniqueId=texture_id, rgbaColor=[1, 1, 1, 1])

    return background


# ============================================================
# GROUND TEXTURE (procedurally generated, seamless grass)
# ============================================================

def _generate_seamless_grass_image(size=512, seed=7):
    """
    Builds a seamless (edge-wrapping) mottled grass texture using layered
    periodic noise, so it tiles across the ground plane with no visible seams.
    Falls back gracefully if numpy/scipy aren't available.
    """
    import numpy as np
    try:
        from scipy.ndimage import gaussian_filter
        wrap_blur = lambda arr, sigma: gaussian_filter(arr, sigma=sigma, mode="wrap")
    except ImportError:
        # Cruder fallback without true edge-wrapping if scipy is missing
        from PIL import Image, ImageFilter

        def wrap_blur(arr, sigma):
            im = Image.fromarray((arr * 255).astype("uint8"))
            im = im.filter(ImageFilter.GaussianBlur(radius=sigma))
            return np.asarray(im).astype("float64") / 255.0

    rng = np.random.RandomState(seed)

    noise = np.zeros((size, size))
    for octave, weight in [(2, 0.5), (6, 0.3), (20, 0.15), (60, 0.05)]:
        layer = rng.rand(size, size)
        layer = wrap_blur(layer, size / octave)
        layer = (layer - layer.min()) / (layer.max() - layer.min() + 1e-9)
        noise += weight * layer
    noise = (noise - noise.min()) / (noise.max() - noise.min() + 1e-9)

    speckle = rng.rand(size, size)
    speckle = wrap_blur(speckle, 1.0)
    speckle = (speckle - speckle.min()) / (speckle.max() - speckle.min() + 1e-9)

    # Muted/desaturated earthy-olive palette (dry grass / scrub trail look).
    # Deliberately kept LOW saturation (verified S <= ~70 on a 0-255 scale)
    # so it stays compatible with perception/free_space.py, which treats
    # low-saturation, reasonably bright pixels as drivable ground. A vivid
    # saturated green would get rejected by that detector as "not free space".
    dark = np.array([55, 60, 45])
    mid = np.array([90, 95, 72])
    light = np.array([140, 142, 115])

    t = noise[..., None]
    color = np.where(t < 0.5, dark + (mid - dark) * (t / 0.5),
                      mid + (light - mid) * ((t - 0.5) / 0.5))

    brightness = 0.9 + 0.2 * speckle[..., None]
    color = np.clip(color * brightness, 0, 255).astype("uint8")

    from PIL import Image
    return Image.fromarray(color, "RGB")


def get_ground_texture_id(project_dir, cache_name="ground_grass_generated.jpg", size=512):
    """
    Returns a PyBullet texture id for a seamless grass ground texture,
    generating and caching it on first run. Falls back to None (default
    checkerboard) if numpy/scipy/Pillow aren't available.
    """
    cache_dir = os.path.join(project_dir, "Assets", "textures")
    cache_path = os.path.join(cache_dir, cache_name)

    if not os.path.exists(cache_path):
        try:
            os.makedirs(cache_dir, exist_ok=True)
            image = _generate_seamless_grass_image(size=size)
            image.save(cache_path, format="JPEG", quality=92)
            print(f"[INFO] Generated seamless ground texture: {cache_path}")
        except Exception as e:
            print(f"[WARNING] Could not generate ground texture ({e}); "
                  "falling back to default checkerboard. "
                  "Run: pip install numpy scipy pillow")
            return None

    texture_id = p.loadTexture(cache_path)
    if texture_id < 0:
        print(f"[WARNING] PyBullet failed to load ground texture: {cache_path}")
        return None

    return texture_id


def apply_ground_terrain(plane_id, texture_id):
    """Applies a texture directly onto the ground plane the UGV drives on."""
    if texture_id is None:
        return
    p.changeVisualShape(plane_id, -1, textureUniqueId=texture_id, rgbaColor=[1, 1, 1, 1])


# ============================================================
# UGV VISUALS - a proper wheeled chassis instead of a flat box
# ============================================================

def create_wheeled_ugv(base_position=(0, 0, 0.35)):
    """
    Creates a UGV with a distinct chassis + 4 visible wheels, so it reads
    as a robot rather than a floating box. Returns the multiBody id.
    """
    body_length, body_width, body_height = 1.6, 1.0, 0.3
    wheel_radius, wheel_width = 0.25, 0.18

    chassis_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[body_length / 2, body_width / 2, body_height / 2]
    )
    chassis_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[body_length / 2, body_width / 2, body_height / 2],
        rgbaColor=[0.15, 0.15, 0.17, 1]  # dark gunmetal chassis, more "real robot"
    )

    wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_width)
    wheel_visual = p.createVisualShape(
        p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_width,
        rgbaColor=[0.05, 0.05, 0.05, 1]  # black tires
    )

    link_positions = [
        [0.55, 0.58, -0.15], [0.55, -0.58, -0.15],
        [-0.55, 0.58, -0.15], [-0.55, -0.58, -0.15]
    ]
    axle_orientation = p.getQuaternionFromEuler([1.5708, 0, 0])

    ugv = p.createMultiBody(
        baseMass=15,
        baseCollisionShapeIndex=chassis_collision,
        baseVisualShapeIndex=chassis_visual,
        basePosition=list(base_position),
        linkMasses=[1, 1, 1, 1],
        linkCollisionShapeIndices=[wheel_collision] * 4,
        linkVisualShapeIndices=[wheel_visual] * 4,
        linkPositions=link_positions,
        linkOrientations=[axle_orientation] * 4,
        linkInertialFramePositions=[[0, 0, 0]] * 4,
        linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
        linkParentIndices=[0, 0, 0, 0],
        linkJointTypes=[p.JOINT_REVOLUTE] * 4,
        linkJointAxis=[[0, 1, 0]] * 4
    )

    # This UGV is driven via resetBaseVelocity (a kinematic-style move), not
    # by spinning these wheel joints. PyBullet enables a default resistive
    # motor on every revolute joint unless told otherwise - left on, the
    # wheels would drag/catch against the ground and cause visible jitter.
    # Free-spin them instead so they roll passively and don't fight the motion.
    for joint_index in range(4):
        p.setJointMotorControl2(
            ugv, joint_index,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )

    return ugv


# ============================================================
# LIGHTING / CAMERA POLISH
# ============================================================

def apply_realistic_scene_settings(camera_target=(0, 0, 0.5), camera_distance=6,
                                    camera_yaw=50, camera_pitch=-30):
    """Shadows + a framed camera angle so the scene reads as 3D, not flat CAD boxes."""
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=list(camera_target)
    )
