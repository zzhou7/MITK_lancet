if(MITK_WRAP_PYTHON_ENABLED)
  message("MITK Python wrapping was successfully enabled")
  include(UseSWIG)

  include(mitkSwigPrepareFiles)

  # Path to common files
  set(MITK_WRAPPING_COMMON_DIR "${MITK_SOURCE_DIR}/Wrapping/Common")

  # Make a manual list of dependencies for the Swig.i files
  list(APPEND SWIG_EXTRA_DEPS
    "${MITK_WRAPPING_COMMON_DIR}/MITK_Common.i"
  )

  # A general packaging target, not built by default, to build packages for each
  # language. This should depend on all language specific targets.
  add_custom_target(dist ${CMAKE_COMMAND} -E echo "Finished generating wrapped packages for distribution...")
  set_property(TARGET dist PROPERTY FOLDER "${MITK_ROOT_FOLDER}/Wrapping")

  add_subdirectory(Python)
endif()
