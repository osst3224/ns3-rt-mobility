build_lib_example(
  NAME simple-sionna-example
  SOURCE_FILES 
  simple-sionna-example.cc
  ue-trajectory.cc
  LIBRARIES_TO_LINK
    ${libsionna}
    ${libcore}
    ${liblte}
    ${libinternet}
    ${libnetwork}
    ${libmobility}
    ${libapplications}
    ${libflow-monitor}
)