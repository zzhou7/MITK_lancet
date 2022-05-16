project(org_mitk_lancet_spine2d3dregistration)

mitk_create_plugin(
  EXPORT_DIRECTIVE SPINE2D3DREGISTRATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt
)
