cmake_minimum_required(VERSION 3.16)
project(octo-foc
        LANGUAGES C
        VERSION 1.0.0.0
        DESCRIPTION "Octo-FOC"
        # HOMEPAGE_URL
)
#project(octofoc C CXX ASM)
set(CMAKE_C_STANDARD 99)

# 指定版本号的配置文件
configure_file(${PROJECT_SOURCE_DIR}/version.h.in ${PROJECT_SOURCE_DIR}/version.h)

#设置输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib) # 设置静态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib) # 动态库文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin) # 可执行文件目录

#设置编译、链接参数等
#set(LINK_FLAGS      "${LINK_FLAGS} -Wl,-Map=object.map,--cref,--gc-section")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdata-sections -ffunction-sections -Wall")

# main add
include_directories(./
                    ./src/foc
                    ./src/pid
                    ./src/qmath
                    foc_port
)

file(GLOB_RECURSE SOURCES
     "./src/foc/*.*"
     "./src/pid/*.*"
     "./src/qmath/*.*"
     "./foc_port/*.*"
)

add_subdirectory(./src/sensor)
add_library(octo-foc STATIC ${SOURCES})
target_link_libraries(octo-foc sensor)
target_include_directories(octo-foc PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${CMAKE_CURRENT_SOURCE_DIR}/src/foc
                           ${CMAKE_CURRENT_SOURCE_DIR}/src/pid
                           ${CMAKE_CURRENT_SOURCE_DIR}/src/qmath
                           ${CMAKE_CURRENT_SOURCE_DIR}/foc_port
)
