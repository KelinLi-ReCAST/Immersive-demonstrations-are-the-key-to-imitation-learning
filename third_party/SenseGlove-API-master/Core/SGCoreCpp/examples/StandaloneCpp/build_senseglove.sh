#!/usr/bin/env bash
# Builds libsenseglove.so (C wrapper around the SenseGlove SDK, loaded from Python via ctypes)
# and copies it, together with the SDK libraries it depends on, into each code/demonstrations/<gripper>/ folder.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CORE="$HERE/../.."                 # SGCoreCpp
CONNECT="$HERE/../../../SGConnect" # SGConnect
REPO="$(cd "$HERE/../../../../../.." && pwd)"

g++ -std=c++11 -O2 -shared -fPIC "$HERE/SenseGlove.cpp" \
    -I"$HERE" -I"$CORE/incl" -I"$CONNECT/incl" \
    -L"$CORE/lib/linux/Release" -L"$CONNECT/lib/linux/Release" \
    -lSGCoreCpp -lSGConnect \
    -Wl,-rpath,'$ORIGIN' \
    -o "$HERE/libsenseglove.so"

for g in mano franka RUTH; do
    dst="$REPO/code/demonstrations/$g"
    cp "$HERE/libsenseglove.so" \
       "$CORE/lib/linux/Release/libSGCoreCpp.so" \
       "$CONNECT/lib/linux/Release/libSGConnect.so" "$dst/"
    echo "installed -> $dst"
done
