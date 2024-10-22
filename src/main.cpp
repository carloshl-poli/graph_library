#include "graph.hpp"
#include "adj_vector.hpp"
#include "adj_matrix.hpp"
#include "structure_base.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>

size_t currentMemoryUsage = 0;

void* operator new(size_t size) {
    currentMemoryUsage += size;
    return malloc(size);
}

void operator delete(void* pointer, size_t size) noexcept {
    currentMemoryUsage -= size;
    free(pointer);
}

void logResults(const std::string& message) {
    std::ofstream logFile("../output/results.log", std::ios::app);
    if (logFile.is_open()) {
        logFile << message << std::endl;
        logFile.close();
    } else {
        std::cerr << "Unable to open log file." << std::endl;
    }
}

int main() {
    int graphNumber;
    std::cout << "Digite o número do grafo (1-6): ";
    std::cin >> graphNumber;

    if (graphNumber < 1 || graphNumber > 6) {
        std::cerr << "Número do grafo inválido." << std::endl;
        return 1;
    }

    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_" + std::to_string(graphNumber) + ".txt";
    std::string file_path = graph_source_folder + graph_file_name;

    // Testes
    Graph* graphVector = new Graph(file_path, GraphStructure::AdjVector, false, false);

    // 1. Comparar uso de memória para AdjVector
    logResults("Uso de memória (bytes) após carregar o grafo com AdjVector: " + std::to_string(currentMemoryUsage));
    double memoryUsageVectorMB = currentMemoryUsage / (1024.0 * 1024.0); // Converter para MB
    std::cout << "Uso de memória (AdjVector): " << memoryUsageVectorMB << " MB" << std::endl;

    // Criar grafo com AdjMatrix para medir custo de memória
    Graph* graphMatrix = new Graph(file_path, GraphStructure::AdjMatrix, false, false);
    
    // 1. Comparar uso de memória para AdjMatrix
    logResults("Uso de memória (bytes) após carregar o grafo com AdjMatrix: " + std::to_string(currentMemoryUsage));
    double memoryUsageMatrixMB = currentMemoryUsage / (1024.0 * 1024.0); // Converter para MB
    std::cout << "Uso de memória (AdjMatrix): " << memoryUsageMatrixMB << " MB" << std::endl;

    // 2. Tempo de execução da busca em largura
    std::cout << "Iniciando teste de busca em largura." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        std::cout << "Executando BFS a partir do vértice " << (i % graphVector->getVertexAmount() + 1) << "..." << std::endl;
        graphVector->getBFSTree(i % graphVector->getVertexAmount() + 1); // Usando diferentes vértices
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    double averageBFS = duration / 100.0;
    logResults("Tempo médio de BFS: " + std::to_string(averageBFS) + " microsegundos.");
    std::cout << "Tempo médio de BFS: " << averageBFS << " microsegundos." << std::endl;
    std::cout << "Teste de busca em largura concluído." << std::endl;

    // 3. Tempo de execução da busca em profundidade
    std::cout << "Iniciando teste de busca em profundidade." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; ++i) {
        std::cout << "Executando DFS a partir do vértice " << (i % graphVector->getVertexAmount() + 1) << "..." << std::endl;
        graphVector->getDFSTree(i % graphVector->getVertexAmount() + 1);
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    double averageDFS = duration / 100.0;
    logResults("Tempo médio de DFS: " + std::to_string(averageDFS) + " microsegundos.");
    std::cout << "Tempo médio de DFS: " << averageDFS << " microsegundos." << std::endl;
    std::cout << "Teste de busca em profundidade concluído." << std::endl;

    // 4. Determinar pais dos vértices 10, 20, 30 na BFS e DFS
    std::cout << "Iniciando teste para determinar pais dos vértices 10, 20 e 30." << std::endl;
    auto bfsTree = graphVector->getBFSTree(1);
    std::cout << "Pai do vértice 10 na BFS: " << bfsTree[10].parent << std::endl;
    logResults("Pai do vértice 10 na BFS: " + std::to_string(bfsTree[10].parent));

    std::cout << "Pai do vértice 20 na BFS: " << bfsTree[20].parent << std::endl;
    logResults("Pai do vértice 20 na BFS: " + std::to_string(bfsTree[20].parent));

    std::cout << "Pai do vértice 30 na BFS: " << bfsTree[30].parent << std::endl;
    logResults("Pai do vértice 30 na BFS: " + std::to_string(bfsTree[30].parent));

    auto dfsTree = graphVector->getDFSTree(1);
    std::cout << "Pai do vértice 10 na DFS: " << dfsTree[10].parent << std::endl;
    logResults("Pai do vértice 10 na DFS: " + std::to_string(dfsTree[10].parent));

    std::cout << "Pai do vértice 20 na DFS: " << dfsTree[20].parent << std::endl;
    logResults("Pai do vértice 20 na DFS: " + std::to_string(dfsTree[20].parent));

    std::cout << "Pai do vértice 30 na DFS: " << dfsTree[30].parent << std::endl;
    logResults("Pai do vértice 30 na DFS: " + std::to_string(dfsTree[30].parent));
    std::cout << "Teste de pais dos vértices concluído." << std::endl;

    // 5. Distâncias entre pares de vértices
    std::cout << "Iniciando teste de distâncias entre pares de vértices." << std::endl;
    auto dist10_20 = graphVector->getUVDistance(10, 20).value();
    std::cout << "Calculando a distância entre 10 e 20..." << std::endl;
    logResults("Distância entre 10 e 20: " + std::to_string(dist10_20));

    auto dist10_30 = graphVector->getUVDistance(10, 30).value();
    std::cout << "Calculando a distância entre 10 e 30..." << std::endl;
    logResults("Distância entre 10 e 30: " + std::to_string(dist10_30));

    auto dist20_30 = graphVector->getUVDistance(20, 30).value();
    std::cout << "Calculando a distância entre 20 e 30..." << std::endl;
    logResults("Distância entre 20 e 30: " + std::to_string(dist20_30));
    std::cout << "Teste de distâncias concluído." << std::endl;

    // 6. Componentes conexas
    std::cout << "Iniciando teste de componentes conexas." << std::endl;
    auto components = graphVector->getGraphSubComp();
    logResults("Número de componentes conexas: " + std::to_string(components.size()));
    std::cout << "Número de componentes conexas: " << components.size() << std::endl;
    // Aqui, você deve calcular o tamanho da maior e menor componente.
    std::cout << "Teste de componentes conexas concluído." << std::endl;

    // 7. Diâmetro do grafo
    std::cout << "Iniciando teste de diâmetro do grafo." << std::endl;
    auto diameter = graphVector->getExactDiameter();
    logResults("Diâmetro do grafo: " + std::to_string(diameter));
    std::cout << "Diâmetro do grafo: " << diameter << std::endl;

    // Limpeza de memória
    delete graphVector;
    delete graphMatrix;

    return 0;
}
