cmake_minimum_required(VERSION 3.28)
project(GeometryKit)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DNOMINMAX)
if (MSVC)
    add_compile_options(/bigobj)
endif ()

file(GLOB imgui_cpp ./include/imgui/*.cpp)
file(GLOB imguizmo_cpp ./include/ImGuizmo/*.cpp)
file(GLOB imguibeckends_cpp ./include/imgui/backends/*.cpp)

add_executable(GeometryKit src/main.cpp
        src/glad.c
        ${imgui_cpp}
        ${imguizmo_cpp}
        ${imguibeckends_cpp}
)

target_link_libraries(GeometryKit "${CMAKE_SOURCE_DIR}/libs/glfw3.lib")

target_include_directories(GeometryKit PUBLIC include)
target_include_directories(GeometryKit PUBLIC include/igl)
target_include_directories(GeometryKit PUBLIC include/Eigen)
target_include_directories(GeometryKit PUBLIC include/glad)
target_include_directories(GeometryKit PUBLIC include/GLFW)
target_include_directories(GeometryKit PUBLIC include/imgui)
target_include_directories(GeometryKit PUBLIC include/ImGuizmo)

