# CVML Challenge Part 2

**Candidate:** N. Goberville | **Date:** April 13, 2026 | **Position:** Computer Vision / ML Engineer

---

## Usage

### 1. Open in Dev Container

Open this repository in VS Code and select **Reopen in Container** when prompted. The container will install all C++ and Python dependencies automatically.

### 2. Build

Compiles the C++ application.

```
just build
```

### 3. Run

Runs the pipeline — loads camera views, cleans masks, backprojects boundaries onto the ground plane, unions them, and writes `outputs/world_mask_boundary.ply`.

```
just run
```

### 4. Visualize

Renders the world mask boundary alongside the scene point cloud and writes an interactive HTML viewer to `outputs/visualize.html`.

```
just visualize
```

Open `outputs/visualize.html` in your browser to explore the result.
