project(org_mitk_lancet_robot)
set(qt5_depends Qt5|Widgets+Network)
mitk_create_plugin(
  EXPORT_DIRECTIVE ROBOT_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkRobot MitkRobotBase
  #PACKAGE_DEPENDS ${qt5_depends}
)
#add_subdirectory(src/api/staubliapi)

#target_link_libraries(org_mitk_lancet_robot PRIVATE staubliapi)