build_lib_example(
  NAME simple-sionna-example
  SOURCE_FILES simple-sionna-example.cc
  LIBRARIES_TO_LINK
    ${libsionna}
    ${libcore}
    ${liblte}
    ${libinternet}
    ${libnetwork}
    ${libmobility}
    ${libapplications}
)

if(${ENABLE_TAP})
  build_lib_example(
    NAME sionna-ns3-to-linux-tap
    SOURCE_FILES sionna-ns3-to-linux-tap.cc
    LIBRARIES_TO_LINK
      ${libsionna}
      ${libcore}
      ${libnetwork}
      ${libinternet}
      ${libmobility}
      ${liblte}
      ${libcsma}
      ${libtap-bridge}
      ${libapplications}
      ${libflow-monitor}
  )

  if(TARGET sionna-ns3-to-linux-tap AND TARGET tap-creator)
    add_dependencies(sionna-ns3-to-linux-tap tap-creator)
  endif()
endif()
