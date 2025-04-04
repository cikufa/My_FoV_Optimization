file(REMOVE_RECURSE
  "libmontecarlo.a"
  "libmontecarlo.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/montecarlo.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
