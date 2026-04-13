"""
Visualize the world mask boundary PLY alongside the scene point cloud.
Opens an interactive 3D scatter plot in your browser — works in devcontainers.

Controls (in the browser):
  Left-click + drag   : rotate
  Right-click + drag  : pan
  Scroll wheel        : zoom
  Double-click        : reset view
"""

import pathlib

import numpy as np
import plotly.graph_objects as go
from plyfile import PlyData

PROJECT_ROOT = pathlib.Path(__file__).resolve().parent.parent

SCENE_PLY = PROJECT_ROOT / "data" / "cloud_and_poses.ply"
MASK_PLY  = PROJECT_ROOT / "outputs" / "world_mask_boundary.ply"


def load_ply(path):
    """Return (x, y, z, r, g, b) arrays from a PLY file."""
    data = PlyData.read(path).elements[0].data
    x = data["x"].astype(float)
    y = data["y"].astype(float)
    z = data["z"].astype(float)
    # colours may be stored as 'red'/'green'/'blue' or 'r'/'g'/'b'
    try:
        r, g, b = data["red"], data["green"], data["blue"]
    except ValueError:
        r, g, b = data["r"], data["g"], data["b"]
    return x, y, z, r, g, b


def rgb_to_plotly(r, g, b):
    """Convert uint8 colour arrays to a list of 'rgb(R,G,B)' strings."""
    return [f"rgb({ri},{gi},{bi})" for ri, gi, bi in zip(r, g, b)]


print("Loading scene cloud ...")
sx, sy, sz, sr, sg, sb = load_ply(SCENE_PLY)
print(f"  {len(sx)} points")

print("Loading mask boundary ...")
mx, my, mz, mr, mg, mb = load_ply(MASK_PLY)
print(f"  {len(mx)} points")

scene_trace = go.Scatter3d(
    x=sx, y=sy, z=sz,
    mode="markers",
    marker=dict(size=2, color=rgb_to_plotly(sr, sg, sb)),
    name="Scene cloud",
)

mask_trace = go.Scatter3d(
    x=mx, y=my, z=mz,
    mode="markers",
    marker=dict(size=3, color=rgb_to_plotly(mr, mg, mb)),
    name="Mask boundary",
)

fig = go.Figure(data=[scene_trace, mask_trace])
fig.update_layout(
    title="World Mask Boundary",
    scene=dict(aspectmode="data"),   # keeps true proportions
    margin=dict(l=0, r=0, t=40, b=0),
)

out_html = PROJECT_ROOT / "outputs" / "visualize.html"
fig.write_html(out_html)
print(f"\nSaved interactive viewer -> {out_html}")
print("Open that file in your browser to explore the point cloud.")
