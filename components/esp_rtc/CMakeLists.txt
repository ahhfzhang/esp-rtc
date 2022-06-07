set(COMPONENT_ADD_INCLUDEDIRS include)

register_component()

target_link_libraries(${COMPONENT_TARGET} INTERFACE "-L ${CMAKE_CURRENT_SOURCE_DIR}/lib")

add_prebuilt_library(esp_rtc "${CMAKE_CURRENT_SOURCE_DIR}/lib/libesp_rtc.a"
                 PRIV_REQUIRES esp-adf-libs)

target_link_libraries(${COMPONENT_TARGET} INTERFACE esp_rtc)
