cmake_minimum_required(VERSION 3.5) #设置CMake最小版本
project (tinyrender) #设置工程名

aux_source_directory(src SRC_SUB)
aux_source_directory(. SRC_CUR)
add_executable(${PROJECT_NAME} ${SRC_SUB} ${SRC_CUR})
include_directories(include ./third_part/eigen)

# target_include_directories(${PROJECT_NAME}
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

