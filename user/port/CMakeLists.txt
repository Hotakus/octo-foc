
include_directories(./
                    ./encoder
                    ./fdcan
                    ./foc_core
)

file(GLOB_RECURSE SOURCES
     "./encoder/*.*"
     "./fdcan/*.*"
     "./foc_core/*.*"
)

add_library(focport STATIC ${SOURCES})
target_include_directories(focport PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/encoder
                           ${CMAKE_CURRENT_SOURCE_DIR}/fdcan
                           ${CMAKE_CURRENT_SOURCE_DIR}/foc_core
)