file(REMOVE_RECURSE
  "libcloud_loader.a"
  "libcloud_loader.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/cloud_loader.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
