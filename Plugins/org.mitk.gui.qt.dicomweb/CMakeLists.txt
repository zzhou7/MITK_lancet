project(org_mitk_gui_qt_dicomweb)

include_directories(${CTK_INCLUDE_DIRS})

mitk_create_plugin(
  EXPORT_DIRECTIVE MITK_QT_DICOMWEB
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkREST MitkRESTService MitkDICOMweb  MitkMultilabel MitkDICOMSegIO
  PACKAGE_DEPENDS Qt5|OpenGL
)



