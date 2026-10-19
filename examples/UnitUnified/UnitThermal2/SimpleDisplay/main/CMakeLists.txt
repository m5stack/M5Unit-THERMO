# Dependencies (M5Unit-THERMO, M5UnitUnified, M5Unified, ...) are pulled in via
# main/idf_component.yml and auto-added as requirements by the component manager.
idf_component_register(
    SRCS "SimpleDisplay.cpp"
    INCLUDE_DIRS ".")
