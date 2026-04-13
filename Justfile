build_dir := "build"

# Build the C++ application
build:
    cmake -B {{build_dir}} -S .
    cmake --build {{build_dir}}

# Run the C++ app to generate the edge map PLY
run: build
    ./{{build_dir}}/CVMLTakehomeChallenge

# Run the Python visualizer to produce the HTML output
visualize:
    uv run python scripts/visualize.py
