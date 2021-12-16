project(org_mitk_lancet_fit)
include_directories(${CTK_INCLUDE_DIRS})
mitk_create_plugin(
  EXPORT_DIRECTIVE FIT_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkLancetAlgo
)
