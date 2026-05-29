file(GLOB_RECURSE APP_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/../lib/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../utils/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/../utils/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../robot/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/../robot/*.cpp"
)

list(FILTER APP_SOURCES EXCLUDE REGEX ".*/example/.*")

idf_component_register(
    SRCS ${APP_SOURCES}
    INCLUDE_DIRS
        "../include"
        "../lib"
        "../utils"
        "../robot"
    REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash driver
)
