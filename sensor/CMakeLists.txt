
include_directories(./
                    ./AS5600
)

file(GLOB_RECURSE SOURCES
     "./AS5600/*.*"
)

add_subdirectory(./mt6835)
add_library(focsensor STATIC ${SOURCES})
target_link_libraries(focsensor PUBLIC mt6835)

target_include_directories(focsensor PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/AS5600
)