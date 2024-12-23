cmake_minimum_required(VERSION 3.10) # Versione minima di CMake
project(BdsSimulation) # Nome del progetto

# Aggiungi i file sorgenti
set(SOURCES
    maingraph.cpp
    boids.hpp
    boids.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=undefined")

# Trova la libreria SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Crea l'eseguibile
add_executable(boids ${SOURCES})

target_compile_options(boids PRIVATE
    -Wall
    -Wextra
    -Wpedantic 
    -Wconversion 
    -Wsign-conversion 
    -Wshadow 
    -Wimplicit-fallthrough 
    -Wextra-semi 
    -Wold-style-cast 
    -D_GLIBCXX_ASSERTIONS 
    -fsanitize=address,undefined
)


# Linka SFML
target_link_libraries(boids sfml-graphics sfml-window sfml-system)
