#!/usr/bin/env bash
# Build the minimal Voxblox core needed to read .vxblx ESDF maps when the old
# ROS/catkin FIF_ws is unavailable.
set -euo pipefail

manifold_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${manifold_dir}/../.." && pwd)"
voxblox_src="${workspace_root}/voxblox_standalone_src"
minkindr_src="${workspace_root}/minkindr_standalone_src"
standalone_build="${workspace_root}/voxblox_standalone_build"

if [[ ! -d "${voxblox_src}/.git" ]]; then
  git clone --depth 1 https://github.com/ethz-asl/voxblox.git "${voxblox_src}"
fi
if [[ ! -d "${minkindr_src}/.git" ]]; then
  git clone --depth 1 https://github.com/ethz-asl/minkindr.git "${minkindr_src}"
fi

mkdir -p "${standalone_build}/generated"
protoc \
  --cpp_out="${standalone_build}/generated" \
  -I "${voxblox_src}/voxblox/proto" \
  "${voxblox_src}/voxblox/proto/voxblox/Block.proto" \
  "${voxblox_src}/voxblox/proto/voxblox/Layer.proto"

common_flags=(
  -std=c++14 -O2 -fPIC -DGLOG_USE_GLOG_EXPORT
  "-I${voxblox_src}/voxblox/include"
  "-I${minkindr_src}/minkindr/include"
  "-I${standalone_build}/generated"
  -I/usr/include/eigen3
)

for source in \
  core/block \
  core/esdf_map \
  utils/protobuf_utils \
  utils/voxel_utils \
  utils/evaluation_utils; do
  object="${standalone_build}/$(basename "${source}").o"
  g++ "${common_flags[@]}" \
    -c "${voxblox_src}/voxblox/src/${source}.cc" \
    -o "${object}"
done

for source in "${standalone_build}"/generated/voxblox/*.pb.cc; do
  object="${standalone_build}/$(basename "${source%.cc}").o"
  g++ "${common_flags[@]}" -c "${source}" -o "${object}"
done

g++ -shared -Wl,-rpath,/usr/local/lib \
  -o "${standalone_build}/libvoxblox_standalone.so" \
  "${standalone_build}/block.o" \
  "${standalone_build}/esdf_map.o" \
  "${standalone_build}/protobuf_utils.o" \
  "${standalone_build}/voxel_utils.o" \
  "${standalone_build}/evaluation_utils.o" \
  "${standalone_build}/Block.pb.o" \
  "${standalone_build}/Layer.pb.o" \
  -L/usr/local/lib -lprotobuf -lglog -lgflags -lpthread

cmake -S "${manifold_dir}" -B "${manifold_dir}/build"
cmake --build "${manifold_dir}/build" --target manifold_test_trajectory -j2

echo "Built ESDF-capable optimizer: ${manifold_dir}/build/manifold_test_trajectory"
