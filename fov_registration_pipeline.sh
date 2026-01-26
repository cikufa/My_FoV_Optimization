#!/usr/bin/env bash
set -euo pipefail

###############################################################################
# Registration pipeline for FOV testing plus optional quiver visualization.
# Adjust the directories if you keep the repo elsewhere.
#
# Usage example:
#   QUIVER_BEFORE=../fovtest/trajectory_before.csv \
#   QUIVER_AFTER=../fovtest/trajectory_after.csv \
#   QUIVER_OUTPUT=./quiver_cmp.png \
#   ./scripts/fov_registration_pipeline.sh
###############################################################################

FOVTEST_DIR=~/fov_ws/my_FIF-perception-aware-planning/act_map_exp/localization/fovtest
COLMAP_SCRIPTS=../colmap_scripts
isaac_dir=~/fov_ws/my_FIF-perception-aware-planning/warehouse_bin/LinuxNoEditor

cd "$isaac_dir"
"./IsaacSimProject.sh" &
sleep 5

cd "$FOVTEST_DIR"
python3 "$COLMAP_SCRIPTS/my_render_ue.py" optimized_stamped_Twc_ue.txt --save_dir ./
python3 "$COLMAP_SCRIPTS/generate_img_rel_path.py" \
  --base_dir ../warehouse_base/images \
  --img_dir ./images \
  --output_dir ./images \
  --img_nm_to_cam_list ./img_nm_to_colmap_cam.txt 

cd ../warehouse_base/
python3 "$COLMAP_SCRIPTS/register_images_to_model.py" ./ \
  --reg_name fov \
  --upref_no_time \
  --reg_list_fn ../fovtest/images/rel_img_path.txt \
  --img_nm_to_colmap_cam_list ../fovtest/images/rel_img_nm_to_cam_list.txt

python3 "$COLMAP_SCRIPTS/calculate_pose_errors.py" \

  --reg_model_dir ./fov_sparse \
  --reg_img_name_to_colmap_Tcw ../fovtest/img_name_to_colmap_Tcw.txt \
  --reg_img_dir ../fovtest/images \
  --output_path ./

if [[ -n "${QUIVER_BEFORE:-}" && -n "${QUIVER_AFTER:-}" ]]; then
  python3 "$COLMAP_SCRIPTS/quiver_viz.py" \
    --before "$QUIVER_BEFORE" \
    --after "$QUIVER_AFTER" \
    --save "${QUIVER_OUTPUT:-quiver_comparison.png}"
fi
