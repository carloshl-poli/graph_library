# Definir a versão mínima do CMake
cmake_minimum_required(VERSION 3.10)

# Nome do projeto
project(GraphLibrary LANGUAGES CXX)

# Definir o tipo de construção como Debug
set(CMAKE_BUILD_TYPE Debug)  # Adicionada aqui

# Habilitar C++11
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Definir os diretórios de inclusão
include_directories(include)

# Detectar o sistema operacional
if (WIN32)
    message(STATUS "Compilando no Windows")
    add_definitions(-DWINDOWS)
elseif (UNIX)
    message(STATUS "Compilando no Linux")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
endif()

# Use file(GLOB ...) to get all .cpp files in the source directory
file(GLOB SOURCES "src/*.cpp")  # Correção aqui


# Adicionar os arquivos de origem ao projeto
# add_executable(graph_project src/main.cpp src/graph.cpp)
add_executable(graph_project ${SOURCES})

# Linking com bibliotecas adicionais se necessário (Windows, por exemplo)
if (WIN32)
    target_link_libraries(graph_project ws2_32)  # exemplo: winsock no Windows
endif()