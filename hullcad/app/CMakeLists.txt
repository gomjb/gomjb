# Qt 6 desktop application (Windows x64 / ARM64, also builds on Linux and macOS).
qt_standard_project_setup()

qt_add_executable(hullcad_app WIN32 MACOSX_BUNDLE
  main.cpp
  MainWindow.cpp MainWindow.h
  YardForm.cpp YardForm.h
  SvgView.cpp SvgView.h
  GzPlot.cpp GzPlot.h
  DailyPlot.cpp DailyPlot.h
  CombPlot.cpp CombPlot.h
  pipeline.cpp pipeline.h
)
set_target_properties(hullcad_app PROPERTIES OUTPUT_NAME hullcad)
target_link_libraries(hullcad_app PRIVATE hullcad Qt6::Widgets Qt6::Svg Qt6::SvgWidgets Qt6::Concurrent)
if(WIN32)
  target_compile_definitions(hullcad_app PRIVATE NOMINMAX UNICODE _UNICODE)
endif()

# `cmake --install` produces a self-contained folder: the app, the Qt DLLs/plugins
# (windeployqt on Windows) and the example yard profiles.
install(TARGETS hullcad_app hullcad_demo
  BUNDLE DESTINATION .
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/yards DESTINATION ${CMAKE_INSTALL_BINDIR})
qt_generate_deploy_app_script(
  TARGET hullcad_app
  OUTPUT_SCRIPT hullcad_deploy_script
  NO_UNSUPPORTED_PLATFORM_ERROR)
install(SCRIPT ${hullcad_deploy_script})
