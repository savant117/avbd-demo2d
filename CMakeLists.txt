cmake_minimum_required(VERSION 3.13)

# Project Name
project(avbd_demo2d)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Automatically collect all source files from the "source" directory
file(GLOB_RECURSE SOURCES "source/*.cpp" "source/*.h")

# Add Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Include SDL2 first (ImGui needs it)
add_subdirectory(external/SDL)

# Add ImGui as a separate project
set(IMGUI_PROJECT_NAME "imgui")

file(GLOB IMGUI_SRC
    external/imgui/*.cpp
    external/imgui/backends/imgui_impl_sdl2.cpp
    external/imgui/backends/imgui_impl_opengl3.cpp
)

add_library(${IMGUI_PROJECT_NAME} STATIC ${IMGUI_SRC})

# Set include directories for ImGui
target_include_directories(${IMGUI_PROJECT_NAME} PUBLIC
    external/imgui
    external/imgui/backends
    external/SDL/include
)

# Link ImGui with SDL2
target_link_libraries(${IMGUI_PROJECT_NAME} PUBLIC SDL2::SDL2 SDL2::SDL2main)

# Link the main project with SDL2 and ImGui
target_include_directories(${PROJECT_NAME} PRIVATE external/SDL/include)
target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2 SDL2::SDL2main ${IMGUI_PROJECT_NAME})

if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -s USE_SDL=2 -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s SINGLE_FILE=1 -s LEGACY_GL_EMULATION=1 --shell-file ../source/shell.html")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
else()
    # Find and Link OpenGL
    find_package(OpenGL REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
endif()

# Copy SDL2.dll to the output folder on Windows
if(WIN32)
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>    # Path to the built SDL2.dll
            $<TARGET_FILE_DIR:${PROJECT_NAME}> # Output directory of the executable
    )
endif()
