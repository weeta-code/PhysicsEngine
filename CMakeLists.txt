cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/Dependencies/include)

set(Sources
    Src/main.cpp
    Src/physics_engine.cpp
    Dependencies/include/src/glad.c  
)

# Link GLFW manually
include_directories(/opt/homebrew/Cellar/glfw/3.4/include)  
link_directories(/opt/homebrew/Cellar/glfw/3.4/lib)         

# Add the executable
add_executable(${PROJECT_NAME} ${Sources})

# Link GLFW library
target_link_libraries(${PROJECT_NAME} /opt/homebrew/Cellar/glfw/3.4/lib/libglfw.dylib)
