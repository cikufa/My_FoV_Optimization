file(REMOVE_RECURSE
  "libmanifold.a"
  "libmanifold.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/manifold.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
