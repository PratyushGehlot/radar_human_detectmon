file(GLOB_RECURSE SRC_FILES "app/*.c")

file(GLOB_RECURSE GUI_SRC_FILES "gui/*.c")

idf_component_register(
    SRCS ${SRC_FILES} ${GUI_SRC_FILES}
    INCLUDE_DIRS "." "gui"
    REQUIRES spiffs driver esp-box-lite lvgl__lvgl esp_wifi esp_netif nvs_flash
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wno-deprecated-declarations")

set_source_files_properties(
    ${SRC_FILES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE) 

spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
