#!/bin/bash
# Compiles the plugin's shaders outside UE: every entry point of the given .usf files (default: all
# of them) under DXC SM 6.6 (HLSL 2018 and 2021) and glslang, the terrain material's Custom-node
# code (VoxelLandMaterial.ush) through two pixel-shader stand-ins (analytic and cached nodes), and
# the ocean material's (VoxelOcean.ush) through a vertex and a pixel stand-in.
#
#   Tools/ShaderCheck/shader_check.sh <plugin root> [VoxelDensityGPU VoxelOceanPage ...]
#
# Needs dxc (DirectXShaderCompiler release 1.8, linux tarball from its GitHub releases; set DXC to
# its bin/dxc) and glslangValidator (apt install glslang-tools). Prints each failing entry point's
# first errors, then "fails=0" or "fails=1". Engine includes are stubbed (Stubs/), so this checks
# the plugin's own code, not the engine's.
set -u
ROOT=$(cd "${1:?plugin root}" && pwd); shift
HERE=$(cd "$(dirname "$0")" && pwd)
DXC=${DXC:-dxc}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
mkdir -p "$WORK/Plugin/VoxelPlugin/Private" "$WORK/Engine/Public" "$WORK/mat"
# Include paths made relative to the work folder in every file (DXC doesn't resolve a nested
# include's absolute path against -I).
for f in "$ROOT"/Shaders/Private/*; do tr -d '\r' < "$f" | sed 's#"/Plugin/#"Plugin/#; s#"/Engine/#"Engine/#' > "$WORK/Plugin/VoxelPlugin/Private/$(basename "$f")"; done
cp "$HERE/Stubs/Engine/Public/Platform.ush" "$WORK/Engine/Public/"
fails=0
check() {   # file entry
  local f=$1 e=$2 out
  for v in 2018 2021; do
    out=$("$DXC" -T cs_6_6 -E "$e" -HV $v -D GROUP_SIZE=4 -D VOXEL_UNDERWATER_THREADS=8 -I "$WORK" "$f" -Fo /dev/null 2>&1 | grep -E "error|warning" | head -5)
    [ -n "$out" ] && { echo "$(basename "$f") $e HLSL $v: $out"; fails=1; }
  done
  out=$(glslangValidator -D -DGROUP_SIZE=4 -DVOXEL_UNDERWATER_THREADS=8 -S comp -e "$e" -I"$WORK" -V "$f" -o "$WORK/x.spv" 2>&1 | grep -iE "error" | head -5)
  [ -n "$out" ] && { echo "glslang $(basename "$f") $e: $out"; fails=1; }
}
# Default: every .usf. VoxelUnderwaterPass includes the engine's Common.ush: Stubs/ has a stand-in
# with the little the pass reads (the view's pre-exposure).
NAMES=${@:-$(cd "$ROOT/Shaders/Private" && ls *.usf | sed 's/\.usf$//')}
mkdir -p "$WORK/Engine/Private"
cp "$HERE/Stubs/Engine/Private/Common.ush" "$WORK/Engine/Private/"
for n in $NAMES; do
  src="$WORK/$n.hlsl"
  sed 's#"/Engine/#"Engine/#; s#"/Plugin/#"Plugin/#' "$WORK/Plugin/VoxelPlugin/Private/$n.usf" > "$src"
  # Entry points: "void Name(" right after a [numthreads(...)] line, and the river build's macro
  # entries.
  for e in $(awk '/^\[numthreads/ { n = 1; next } n && match($0, /^void [A-Za-z0-9_]+\(/) { print substr($0, 6, RLENGTH - 6) } { n = 0 }' "$src"; grep -oP "^VOXEL_\w*ENTRY\(\K\w+" "$src"); do check "$src" "$e"; done
done
# The material's Custom-node code: the includes made local.
for f in "$WORK"/Plugin/VoxelPlugin/Private/*; do sed 's#"Plugin/VoxelPlugin/Private/#"#' "$f" > "$WORK/mat/$(basename "$f")"; done
cp "$HERE"/Material/*.hlsl "$HERE"/Material/stub_platform.ush "$WORK/mat/"
for t in material_test material_cached; do
  for v in 2018 2021; do
    out=$("$DXC" -T ps_6_6 -E main -HV $v -I "$WORK/mat" "$WORK/mat/$t.hlsl" -Fo /dev/null 2>&1 | grep -E "error|warning" | head -5)
    [ -n "$out" ] && { echo "$t HLSL $v: $out"; fails=1; }
  done
  out=$(glslangValidator -D -S frag -e main -I"$WORK/mat" -V "$WORK/mat/$t.hlsl" -o "$WORK/m.spv" 2>&1 | grep -iE "error" | head -5)
  [ -n "$out" ] && { echo "glslang $t: $out"; fails=1; }
done
# The ocean material's Custom-node code: vertex and pixel stand-ins.
for t in ocean_material_vs:vs_6_6:vert ocean_material_ps:ps_6_6:frag; do
  n=${t%%:*}; rest=${t#*:}; prof=${rest%%:*}; stage=${rest#*:}
  for v in 2018 2021; do
    out=$("$DXC" -T $prof -E main -HV $v -I "$WORK/mat" "$WORK/mat/$n.hlsl" -Fo /dev/null 2>&1 | grep -E "error|warning" | head -5)
    [ -n "$out" ] && { echo "$n HLSL $v: $out"; fails=1; }
  done
  out=$(glslangValidator -D -S $stage -e main -I"$WORK/mat" -V "$WORK/mat/$n.hlsl" -o "$WORK/m.spv" 2>&1 | grep -iE "error" | head -5)
  [ -n "$out" ] && { echo "glslang $n: $out"; fails=1; }
done
# UE's preprocessor pastes a macro's text without spacing: a macro defined as a bare negative number
# right after a minus ("-NAME") becomes "--9.0f", a decrement (DXC spaces it, so the compiles above
# can't see it). Such macros must be parenthesised.
for name in $(grep -hoP '^\s*#define\s+\K\w+(?=\s+-[0-9.])' "$ROOT"/Shaders/Private/*); do
  out=$(grep -nE -- "-$name\b" "$ROOT"/Shaders/Private/* | head -3)
  [ -n "$out" ] && { echo "macro $name is a bare negative number used after a minus (parenthesise it): $out"; fails=1; }
done
echo "fails=$fails"
