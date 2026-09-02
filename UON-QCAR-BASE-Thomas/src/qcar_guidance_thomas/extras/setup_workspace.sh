#!/bin/bash
# ============================================================
# One-time setup after pasting qcar_guidance_thomas into a
# fresh UON-QCAR-BASE workspace. Run from the WORKSPACE ROOT:
#
#     cd <workspace>          (folder containing src/)
#     bash src/qcar_guidance_thomas/extras/setup_workspace.sh
#
# What it does:
#   1. Makes all package scripts executable
#   2. Creates Track1ExampleThomas.launch (copy of the stock
#      Track1Example.launch with the guidance node swapped to
#      this package + params loaded)
#   3. Patches depth_cone_detector_node.py to skip point-cloud
#      frames with an empty frame_id (startup race crash fix)
# ============================================================
set -e

if [ ! -d src ]; then
    echo "ERROR: run this from the workspace root (the folder containing src/)"
    exit 1
fi

echo "[1/3] Making scripts executable..."
chmod +x src/qcar_guidance_thomas/scripts/*.py

echo "[2/3] Creating Track1ExampleThomas.launch..."
STOCK=src/qcar_gazebo/launch/Track1Example.launch
MINE=src/qcar_gazebo/launch/Track1ExampleThomas.launch
if [ ! -f "$STOCK" ]; then
    echo "ERROR: $STOCK not found -- is qcar_gazebo present?"
    exit 1
fi
cp "$STOCK" "$MINE"
python3 - << 'EOF'
import re
path = "src/qcar_gazebo/launch/Track1ExampleThomas.launch"
with open(path) as f:
    src = f.read()

new_node = ('<node name="guidance_node" pkg="qcar_guidance_thomas" '
            'type="online_guidance_node.py" output="screen">'
            '<rosparam command="load" '
            'file="$(find qcar_guidance_thomas)/config/guidance_params.yaml"/>'
            '</node>')

# replace the stock guidance node line (self-closing or not) with ours
pattern = r'<node[^>]*name="guidance_node"[^>]*pkg="qcar_guidance"[^>]*/>'
if re.search(pattern, src):
    src = re.sub(pattern, new_node, src, count=1)
else:
    raise SystemExit("ERROR: stock guidance_node line not found in "
                     "Track1Example.launch -- edit it manually (see README).")
with open(path, "w") as f:
    f.write(src)
print("  Track1ExampleThomas.launch created OK")
EOF

echo "[3/3] Patching depth_cone_detector_node.py (empty frame_id guard)..."
python3 - << 'EOF'
path = "src/qcar_navigation/scripts/depth_cone_detector_node.py"
try:
    with open(path) as f:
        src = f.read()
except FileNotFoundError:
    print("  depth_cone_detector_node.py not found -- skipping (fine if "
          "navigation package absent)")
    raise SystemExit(0)

guard = "if not source_frame:"
if guard in src:
    print("  already patched -- skipping")
    raise SystemExit(0)

old = "        pts_cam = np.array(raw_points, dtype=float)"
new = ("        pts_cam = np.array(raw_points, dtype=float)\n\n"
       "        if not source_frame:\n"
       "            # sensor not fully initialised yet (empty frame_id) -- skip\n"
       "            return\n")
if old not in src:
    print("  WARNING: patch marker not found -- depth node code differs; "
          "patch manually if the TF crash appears")
    raise SystemExit(0)
src = src.replace(old, new, 1)
with open(path, "w") as f:
    f.write(src)
print("  depth node patched OK")
EOF

echo ""
echo "Setup complete. Next steps:"
echo "  catkin_make"
echo "  source devel/setup.bash"
echo "  roslaunch qcar_guidance_thomas test_guidance_isolated.launch   (quick test)"
echo "  roslaunch qcar_guidance_thomas guidance_gazebo.launch          (full sim)"
