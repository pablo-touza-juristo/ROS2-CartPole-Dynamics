cmake_minimum_required(VERSION 3.10)
project(cartpole_physics_validation)

# 1. Estándar C++ (Lo forzamos globalmente aquí)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Flags de compilación (Para que seas riguroso)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. Dependencias (Solo Eigen3 por ahora)
find_package(Eigen3 REQUIRED)

# 4. Librería de Física (Tu motor)
# NOTA: He añadido rk4_integration.cpp que faltaba en tu lista
add_library(physics_engine
  src/dynamics/cartpole.cpp
  src/math/rk4_integration.cpp
)

# Dónde están los headers (.hpp)
target_include_directories(physics_engine PUBLIC 
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# Enlazar Eigen a la librería
target_link_libraries(physics_engine PUBLIC Eigen3::Eigen)

# 5. Ejecutable de Validación (Tu main.cpp)
# Asegúrate de crear un archivo 'src/main.cpp' que NO tenga nada de ROS
add_executable(run_simulation src/cartpole_node.cpp)

# Enlazar tu ejecutable con tu librería de física
target_link_libraries(run_simulation PRIVATE physics_engine)
