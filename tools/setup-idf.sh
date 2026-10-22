#!/usr/bin/env bash
# Linux/macOS ESP32 build tools. Python and all Python packages are managed by uv.
set -euo pipefail

rover_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
idf_commit=30aaf64524299d3bde422ca9a2848090d1bc5d0f
export IDF_PATH="$rover_root/.esp-idf"
export IDF_TOOLS_PATH="$rover_root/.espressif"
export IDF_PYTHON_ENV_PATH="$rover_root/.idf-python"
export UV_PROJECT_ENVIRONMENT="$IDF_PYTHON_ENV_PATH"

command -v uv >/dev/null
command -v git >/dev/null
if [[ ! -d "$IDF_PATH/.git" ]]; then
    if [[ -e "$IDF_PATH" ]]; then
        printf 'Refusing to replace existing non-Git SDK directory: %s\n' "$IDF_PATH" >&2
        exit 1
    fi
    git clone --branch v5.5.2 --depth 1 --recursive --shallow-submodules \
        https://github.com/espressif/esp-idf.git "$IDF_PATH"
fi
if [[ "$(git -C "$IDF_PATH" rev-parse HEAD)" != "$idf_commit" ]]; then
    printf 'SDK revision differs from pinned ESP-IDF v5.5.2; refusing to change it.\n' >&2
    exit 1
fi
git -C "$IDF_PATH" submodule update --init --recursive --depth 1

uv sync --project "$rover_root/tools/idf-python" --locked --python 3.12
# The SDK's compatibility check identifies its Python environment by major/minor.
printf '5.5\n' > "$IDF_PYTHON_ENV_PATH/idf_version.txt"
mkdir -p "$IDF_TOOLS_PATH"
# Use the checked-in constraints snapshot rather than a mutable remote file.
cp "$rover_root/tools/idf-python/espidf.constraints.v5.5.txt" \
    "$IDF_TOOLS_PATH/espidf.constraints.v5.5.txt"
uv run --project "$rover_root/tools/idf-python" --no-sync \
    "$IDF_PATH/tools/idf_tools.py" install --targets esp32 required cmake ninja
uv run --project "$rover_root/tools/idf-python" --no-sync \
    "$IDF_PATH/tools/idf_tools.py" check-python-dependencies
"$rover_root/tools/idf.sh" --version
