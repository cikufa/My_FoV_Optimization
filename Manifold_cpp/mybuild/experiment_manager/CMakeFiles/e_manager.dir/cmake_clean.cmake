file(REMOVE_RECURSE
  "libe_manager.a"
  "libe_manager.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/e_manager.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
