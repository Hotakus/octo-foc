
include_directories(./
                    ./AS5600
                    ./MT6835
)

file(GLOB_RECURSE SOURCES
     "./AS5600/*.*"
     "./MT6835/*.*"
)

add_library(sensor STATIC ${SOURCES})
target_include_directories(sensor PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/AS5600
                           ${CMAKE_CURRENT_SOURCE_DIR}/MT6835
)