project(org_mitk_gui_qt_dicomweb)

mitk_create_plugin(
  EXPORT_DIRECTIVE DICOMWEB_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkREST MitkRESTService MitkDICOMweb MitkChart MitkSegmentationUI MitkDICOMSegIO
  PACKAGE_DEPENDS PUBLIC CTK|CTKWidgets
)



