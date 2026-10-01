#!/bin/bash
# SessionStart hook for Claude Code cloud sessions: installs the Zig toolchain used by the port.
set -euo pipefail

# Only needed in remote (cloud) sessions; local machines manage their own Zig install
if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
    exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(dirname "$0")/../..}"

# Install Zig 0.16.0 (idempotent: exits early when the right version is already on PATH)
bash tools/install_zig.sh

# Warm the Zig cache (std, compiler_rt, the library) so the first `zig build test` is fast.
# Don't fail the session if the tree currently doesn't compile.
zig build check >/dev/null 2>&1 || echo "session-start: 'zig build check' failed, run it to see the errors"
