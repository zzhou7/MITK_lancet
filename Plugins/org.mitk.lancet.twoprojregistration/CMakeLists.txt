project(org_mitk_lancet_twoprojregistration)

mitk_create_plugin(
  EXPORT_DIRECTIVE TWOPROJREGISTRATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkBoundingShape MitkLancetAlgo
)
