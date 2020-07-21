project(org_mitk_gui_qt_bonesegmentation)

mitk_create_plugin(
  EXPORT_DIRECTIVE BONESEGMENTATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkBoneSegmentation
)
