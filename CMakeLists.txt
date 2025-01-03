cmake_minimum_required(VERSION 3.10) # Versione minima di CMake
project(BdsSimulation) # Nome del progetto


#specifica che l'eseguibile binario va nella cartella bin 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)



# Attiva debug e AddressSanitizer
set(SANITIZE_FLAGS "-fsanitize=address,undefined -g -D_GLIBCXX_ASSERTIONS")

# Aggiungi i flag globali
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SANITIZE_FLAGS}")


# Aggiungi i file sorgenti
set(SOURCES
    maingraph.cpp
    boids.cpp
    operators.cpp
    graphics.cpp
    statistics.cpp
    )

#specifica che l'eseguibile binario va nella cartella bin 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

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
