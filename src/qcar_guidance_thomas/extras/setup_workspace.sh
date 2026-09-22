#!/bin/bash
# ============================================================
# One-time setup after putting qcar_guidance_thomas into a
# UON-QCAR-BASE workspace. Run from the WORKSPACE ROOT
# (the folder that contains src/):
#
#     bash src/qcar_guidance_thomas/extras/setup_workspace.sh
#
#   1. makes the node scripts executable
#   2. checks numpy / scipy are installed (installs scipy if not)
#   3. applies the empty-frame_id guard to depth_cone_detector_node.py
#      if it is not already there (Ethan has merged it upstream)
#   4. runs a quick self-test of the guidance core (no ROS needed)
#
# v5 no longer generates Track1ExampleThomas.launch: use
#     roslaunch qcar_guidance_thomas guidance_gazebo.launch track:=Track1
# ============================================================
set -e
if [ ! -d src ]; then
    echo "ERROR: run this from the workspace root (the folder containing src/)"
    exit 1
fi
PKG=src/qcar_guidance_thomas

echo "[1/4] Making scripts executable..."
chmod +x $PKG/scripts/*.py $PKG/extras/*.py $PKG/extras/*.sh

echo "[2/4] Checking Python dependencies..."
if ! python3 -c "import numpy, scipy.sparse, scipy.sparse.linalg" 2>/dev/null; then
    echo "  scipy missing -- installing python3-scipy"
    apt-get install -y python3-scipy
fi
python3 -c "import numpy, scipy; print('  numpy %s, scipy %s OK' % (numpy.__version__, scipy.__version__))"

echo "[3/4] Checking depth_cone_detector_node.py empty frame_id guard..."
python3 - << 'PYEOF'
path = "src/qcar_navigation/scripts/depth_cone_detector_node.py"
try:
    src = open(path).read()
except FileNotFoundError:
    print("  not found -- skipping"); raise SystemExit(0)
if "if not source_frame:" in src:
    print("  already present -- nothing to do"); raise SystemExit(0)
old = "        pts_cam = np.array(raw_points, dtype=float)"
if old not in src:
    print("  WARNING: code differs, patch manually if the TF crash appears"); raise SystemExit(0)
src = src.replace(old, old + "\n\n        if not source_frame:\n            return\n", 1)
open(path, "w").write(src)
print("  patched")
PYEOF

echo "[4/4] Self-test of the guidance core (race line on the real Track1 cones)..."
python3 - << 'PYEOF'
import sys, math
sys.path.insert(0, "src/qcar_guidance_thomas/scripts")
import guidance_core as gc, track_sim as ts
cones = ts.load_world_cones("src/qcar_gazebo/worlds/Track1.world")
blue = [(x, y) for x, y, c in cones if c == ts.BLUE]
yel = [(x, y) for x, y, c in cones if c == ts.YELLOW]
chain, rest = [blue[0]], blue[1:]
while rest:
    q = min(rest, key=lambda r: math.hypot(r[0]-chain[-1][0], r[1]-chain[-1][1]))
    chain.append(q); rest.remove(q)
ref = []
for b in chain:
    o = min(yel, key=lambda q: math.hypot(q[0]-b[0], q[1]-b[1]))
    ref.append(((b[0]+o[0])/2, (b[1]+o[1])/2))
race, why = gc.build_race_line(ref, cones, dict(gc.DEFAULTS))
if race is None:
    print("  SELF-TEST FAILED:", why); raise SystemExit(1)
m = race.metrics
print("  OK: race line %.1f m (centreline %.1f m), clearance %.2f m"
      % (m['lap_length_m'], m['centreline_length_m'], m['min_clearance_m']))
PYEOF

echo ""
echo "Setup complete. Next:"
echo "  rm -rf build devel        (only if build/devel came from someone else's machine)"
echo "  catkin_make && source devel/setup.bash"
echo "  roslaunch qcar_guidance_thomas test_guidance_isolated.launch        # quick, no Gazebo"
echo "  roslaunch qcar_guidance_thomas guidance_gazebo.launch track:=Track1 # full sim"
