file(REMOVE_RECURSE
  "bin/libfuzzylite-static.pdb"
  "bin/libfuzzylite-static.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/fl-static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
