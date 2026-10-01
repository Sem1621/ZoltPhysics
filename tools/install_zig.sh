#!/usr/bin/env bash
# Installs Zig 0.16.0 into /opt/zig-0.16.0 and links it to /usr/local/bin/zig (or $ZIG_BIN_DIR).
#
# ziglang.org may be unreachable from sandboxed environments, so the official binaries are taken
# from the `ziglang` package on PyPI, which wraps the exact release tarballs.
set -euo pipefail

ZIG_VERSION="0.16.0"
INSTALL_DIR="${ZIG_INSTALL_DIR:-/opt/zig-$ZIG_VERSION}"
BIN_DIR="${ZIG_BIN_DIR:-/usr/local/bin}"

if command -v zig >/dev/null 2>&1 && [ "$(zig version)" = "$ZIG_VERSION" ]; then
    exit 0
fi

case "$(uname -m)" in
    x86_64) PLATFORM="manylinux_2_12_x86_64" ;;
    aarch64) PLATFORM="manylinux_2_17_aarch64" ;;
    *) echo "install_zig.sh: unsupported architecture $(uname -m)" >&2; exit 1 ;;
esac

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

python3 -m pip download --quiet "ziglang==$ZIG_VERSION" --no-deps --only-binary=:all: \
    --platform "$PLATFORM" -d "$TMP_DIR"
python3 -m zipfile -e "$TMP_DIR"/ziglang-*.whl "$TMP_DIR/extracted"

rm -rf "$INSTALL_DIR"
mkdir -p "$(dirname "$INSTALL_DIR")" "$BIN_DIR"
mv "$TMP_DIR/extracted/ziglang" "$INSTALL_DIR"
chmod +x "$INSTALL_DIR/zig"
ln -sf "$INSTALL_DIR/zig" "$BIN_DIR/zig"

echo "Installed zig $(zig version) to $INSTALL_DIR"
