# Definir a versão mínima do CMake
cmake_minimum_required(VERSION 3.10)

# Nome do projeto
project(GraphLibrary LANGUAGES CXX)

# Habilitar C++11
set(CMAKE_CXX_STANDARD 11)
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

# Adicionar os arquivos de origem ao projeto
add_executable(graph_project src/main.cpp src/graph.cpp)

# Linking com bibliotecas adicionais se necessário (Windows, por exemplo)
if (WIN32)
    target_link_libraries(graph_project ws2_32)  # exemplo: winsock no Windows
endif()