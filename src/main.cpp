
#include "graph.hpp"
#include "adj_vector.hpp"
#include "adj_matrix.hpp"
#include "structure_base.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>

size_t currentMemoryUsage = 0;
/*
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


*/
int main(){
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "teste_peso.txt";
    std::string file_path = graph_source_folder + graph_file_name;
    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true);
    std::cout << "Graph Initialized" << std::endl;
    auto dist1_3 = graph->getUVDistance(1,3, true);
    std::cout << "Distancia entre 1 e 3 deve ser 7: " << std::to_string(dist1_3) << std::endl;
    return 0;
}


/*



#include "pairing_heap.hpp"
#include <iostream>
#include <cassert>

void testInsertAndFindMin() {
    PairingHeap heap;
    heap.insert(1, 10);
    heap.insert(2, 5);
    heap.insert(3, 20);

    assert(heap.findMin() == 2);
    std::cout << "Test Insert and FindMin Passed!" << std::endl;
}

void testDeleteMin() {
    PairingHeap heap;
    heap.insert(1, 10);
    heap.insert(2, 5);
    heap.insert(3, 20);

    heap.deleteMin();
    assert(heap.findMin() == 1);
    
    heap.deleteMin();
    assert(heap.findMin() == 3);

    heap.deleteMin();
    assert(heap.isEmpty());
    std::cout << "Test DeleteMin Passed!" << std::endl;
}

void testDecreaseKey() {
    PairingHeap heap;
    heap.insert(1, 10);
    heap.insert(2, 5);
    heap.insert(3, 20);
    heap.insert(4, 15);

    // Decrease key for vertex 3, which should now become the minimum
    heap.decreaseKey(3, 2);
    assert(heap.findMin() == 3);

    // Further decrease the key for vertex 1
    heap.decreaseKey(1, 1);
    assert(heap.findMin() == 1);
    std::cout << "Test DecreaseKey Passed!" << std::endl;
}

void testIsEmpty() {
    PairingHeap heap;
    assert(heap.isEmpty());

    heap.insert(1, 10);
    assert(!heap.isEmpty());

    heap.deleteMin();
    assert(heap.isEmpty());
    std::cout << "Test IsEmpty Passed!" << std::endl;
}

void testClear() {
    PairingHeap heap;
    heap.insert(1, 10);
    heap.insert(2, 5);
    heap.insert(3, 20);

    heap.clear();
    assert(heap.isEmpty());
    std::cout << "Test Clear Passed!" << std::endl;
}

int main() {
    testInsertAndFindMin();
    testDeleteMin();
    testDecreaseKey();
    testIsEmpty();
    testClear();

    std::cout << "All tests passed!" << std::endl;
    return 0;
}
*/

/*
#include "graph.hpp"
#include <iostream>
#include <vector>
#include <utility> // Para std::pair
#include <stdexcept> // Para std::logic_error

// Supondo que AdjVector e o método getAdjWeightedArray já estejam definidos.






int main() {

    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "teste_peso.txt";
    std::string file_path = graph_source_folder + graph_file_name;
    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true);
    std::cout << "Graph Initialized" << std::endl;
    int U;

    while (true) {
        std::cout << "Digite o índice do vértice (U) ou -1 para sair: ";
        std::cin >> U;

        // Condição de saída
        if (U == -1) {
            break;
        }

        try {
            std::vector<std::pair<int, double>> adjList = graph->structure->getAdjWeightedArray(U);
            
            std::cout << "Vértices adjacentes e seus pesos:\n";
            for (const auto& pair : adjList) {
                std::cout << "Vértice: " << pair.first << ", Peso: " << pair.second << '\n';
            }
        } catch (const std::logic_error& e) {
            std::cerr << "Erro: " << e.what() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "Ocorreu um erro: " << e.what() << '\n';
        }
    }

    std::cout << "Programa encerrado." << std::endl;
    return 0;
}

*/