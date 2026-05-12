cmake_minimum_required(VERSION 3.14)
project(GR_Simulation CXX)

# Force C++20 standard
set(CMAKE_CXX_STANDARD 20)

# Enable FetchContent to download libraries automatically from GitHub
include(FetchContent)

# 1. Fetch Raylib
FetchContent_Declare(raylib 
    GIT_REPOSITORY https://github.com/raysan5/raylib.git 
    GIT_TAG master
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

# 2. Fetch ImGui
FetchContent_Declare(imgui 
    GIT_REPOSITORY https://github.com/ocornut/imgui.git 
    GIT_TAG master
)
FetchContent_Populate(imgui)

# 3. Fetch rlImGui (The bridge between Raylib and ImGui)
FetchContent_Declare(rlimgui 
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git 
    GIT_TAG main
)
FetchContent_Populate(rlimgui)

# Gather all the ImGui and rlImGui source code files
file(GLOB IMGUI_SOURCES 
    "${imgui_SOURCE_DIR}/*.cpp"
    "${rlimgui_SOURCE_DIR}/*.cpp"
)

# Create the executable program from our main.cpp and the ImGui files
add_executable(${PROJECT_NAME} main.cpp ${IMGUI_SOURCES})

# Tell the compiler where to find the library headers
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${imgui_SOURCE_DIR} 
    ${rlimgui_SOURCE_DIR}
)

# Link Raylib to our project
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)