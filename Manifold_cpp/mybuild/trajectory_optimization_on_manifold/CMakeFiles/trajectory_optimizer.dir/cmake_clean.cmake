file(REMOVE_RECURSE
  "libtrajectory_optimizer.a"
  "libtrajectory_optimizer.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/trajectory_optimizer.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
