"""
Shared terrain/ground texturing helpers for the UGV simulation.

Both camera.py and simulation.py used to have their own copy-pasted version
of this logic, which meant fixing a bug in one file didn't fix the other.
Import from here instead so there's a single source of truth.
"""

import os
import pybullet as p

MAX_TEXTURE_DIM = 2048  # safe upper bound for GPU texture size / stb_image


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
            is_progressive = "progressive" in (im.info.get("jpeg") or "") \
                or getattr(im, "info", {}).get("progression", False)

            needs_resize = width > MAX_TEXTURE_DIM or height > MAX_TEXTURE_DIM
            needs_reencode = needs_resize or im.format == "JPEG"

            # If a valid cache already exists and matches current settings, reuse it
            if os.path.exists(cached_path) and not needs_resize:
                return image_path  # small enough already, no cache needed

            if needs_resize or is_progressive:
                im = im.convert("RGB")
                if needs_resize:
                    im.thumbnail((MAX_TEXTURE_DIM, MAX_TEXTURE_DIM), Image.LANCZOS)
                im.save(cached_path, format="JPEG", quality=90, progressive=False)
                print(f"[INFO] Terrain image was {width}x{height} "
                      f"(too large or progressive) - resized copy saved to: {cached_path}")
                return cached_path

    except Exception as e:
        print(f"[WARNING] Could not inspect/resize terrain image ({e}); "
              "using original file as-is.")

    return image_path


def get_terrain_texture_id(project_dir, sub_path=("Assets", "backgrounds", "terrain.jpg")):
    """Loads the terrain image (auto-resizing if needed) and returns a PyBullet texture id, or None."""
    image_path = os.path.join(project_dir, *sub_path)

    print("Loading terrain texture:", image_path)

    if not os.path.exists(image_path):
        print(f"[WARNING] Terrain texture not found at: {image_path}")
        return None

    safe_path = _get_cached_or_resized_path(image_path)

    texture_id = p.loadTexture(safe_path)

    if texture_id < 0:
        print(f"[WARNING] PyBullet failed to load texture: {safe_path}")
        return None

    return texture_id


def apply_ground_terrain(plane_id, texture_id):
    """Applies the terrain image directly onto the ground plane the UGV drives on."""
    if texture_id is None:
        return
    p.changeVisualShape(
        plane_id,
        -1,
        textureUniqueId=texture_id,
        rgbaColor=[1, 1, 1, 1]  # reset tint so the texture shows its real colors
    )


def create_backdrop_wall(texture_id, position=(45, 0, 20), half_extents=(0.1, 60, 20)):
    """Distant backdrop wall so the horizon also looks like terrain, not empty void."""
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(half_extents))
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=list(half_extents))

    background = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=list(position)
    )

    if texture_id is not None:
        p.changeVisualShape(
            background,
            -1,
            textureUniqueId=texture_id,
            rgbaColor=[1, 1, 1, 1]
        )

    return background
