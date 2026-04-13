#!/bin/bash

set -e

echo "Installing system dependencies..."

sudo apt-get update

# C++ build tools and libraries
sudo apt-get install -y cmake build-essential g++ libeigen3-dev libopencv-dev

# just (command runner)
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | sudo bash -s -- --to /usr/local/bin

echo "System dependencies installed."

echo "Installing uv..."
if curl -LsSf https://astral.sh/uv/install.sh | sh; then
    export PATH="$HOME/.local/bin:$PATH"
    # Persist PATH for bash and zsh
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
else
    echo "astral.sh unreachable, installing uv via pip..."
    pip install uv
fi

echo "Syncing Python dependencies..."
cd /workspaces/cvml_takehome_challenge_pt2
uv sync

echo "All dependencies ready. Run 'just build', 'just run', or 'just visualize'."
