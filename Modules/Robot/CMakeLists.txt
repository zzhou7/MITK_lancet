set(qt5_depends Qt5|Widgets+Network)

MITK_CREATE_MODULE(
  DEPENDS MitkCore
  PACKAGE_DEPENDS PUBLIC ${qt5_depends}
  FORCE_STATIC
)

#add_subdirectory(autoload)