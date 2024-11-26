
#include "graph.hpp"
#include "flow_graph.hpp"
#include "adj_vector.hpp"
#include "adj_matrix.hpp"
#include "structure_base.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdlib>

void studyCaseShortestPathBetweenVerticesGraphOLD(){
    std::cout << "Starting study of Shortest Path Between Vertices and 10 for graph 1" << std::endl;
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_W_1.txt";
    std::string file_path = graph_source_folder + graph_file_name;
    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true);
    std::cout << "Graph Initialized" << std::endl;
    std::vector<int> destinations = {20,30,40,50,60};
    auto dijkstraTree10 = graph->getDijkstraTree(10, true);

    for (const auto& destination : destinations) {
        auto dist = graph->getDijkstraMinDistance(dijkstraTree10, destination);
        if (dist == std::numeric_limits<double>::max()) {
            std::cout << "Não há caminho entre 10 e " << destination << std::endl;
        } else {
            std::cout << "Distancia entre 10 e " << destination << " é " << dist << std::endl;
            std::cout << "Caminho entre 10 e " << destination << " é: ";
            auto deque = graph->getDijkstraMinPathVertices(10, destination, dijkstraTree10);
            while (!deque.empty()) {
                std::cout << deque.back();
                deque.pop_back();
                if (!deque.empty()) {
                    std::cout << " -> ";
                }
            }
        }
        std::cout << std::endl;
    }
}

void studyCaseShortestPathBetweenVerticesGraph(int graph_number) {
    // Define o caminho do log
    std::string log_source_folder = "../output/";
    std::string log_file_name = "log_graph_" + std::to_string(graph_number) + ".txt";
    std::string log_file_path = log_source_folder + log_file_name;
    
    // Abre o arquivo de log para escrita
    std::ofstream log_file(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo de log em " << log_file_path << std::endl;
        return;
    }

    // Caminho do grafo
    std::cout << "Starting study of Shortest Path Between Vertices and 10 for graph " << graph_number << std::endl;
    log_file << "Starting study of Shortest Path Between Vertices and 10 for graph " << graph_number << std::endl;
    
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_W_" + std::to_string(graph_number) + ".txt";
    std::string file_path = graph_source_folder + graph_file_name;

    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true);
    std::cout << "Graph " << graph_number << " Initialized" << std::endl;
    log_file << "Graph " << graph_number << " Initialized" << std::endl;

    std::vector<int> destinations = {20, 30, 40, 50, 60};
    auto dijkstraTree10 = graph->getDijkstraTree(10, true);

    for (const auto& destination : destinations) {
        auto dist = graph->getDijkstraMinDistance(dijkstraTree10, destination);
        if (dist == std::numeric_limits<double>::max()) {
            std::cout << "No path between 10 and " << destination << " in graph " << graph_number << std::endl;
            log_file << "No path between 10 and " << destination << " in graph " << graph_number << std::endl;
        } else {
            std::cout << "Distance between 10 and " << destination << " in graph " << graph_number << " is " << dist << std::endl;
            log_file << "Distance between 10 and " << destination << " in graph " << graph_number << " is " << dist << std::endl;
            std::cout << "Path between 10 and " << destination << " is: ";
            log_file << "Path between 10 and " << destination << " is: ";

            auto deque = graph->getDijkstraMinPathVertices(10, destination, dijkstraTree10);
            while (!deque.empty()) {
                std::cout << deque.back();
                log_file << deque.back();
                deque.pop_back();
                if (!deque.empty()) {
                    std::cout << " -> ";
                    log_file << " -> ";
                }
            }
            std::cout << std::endl;
            log_file << std::endl;
        }
        std::cout << std::endl;
        log_file << std::endl;
    }

    // Fecha o arquivo de log
    log_file.close();
}



void studyCaseAverageDijkstraExecutionTime(){

}


void studyCaseCollaborationPathWithDijkstraOLD(){
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "rede_colaboracao.txt";
    std::string graph_names_file_name = "rede_colaboracao_vertices.txt";
    std::string file_path = graph_source_folder + graph_file_name;
    std::string name_path = graph_source_folder + graph_names_file_name;
    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true, name_path);
    std::cout << "Graph Initialized" << std::endl;

    std::vector<std::string> pesquisadores = {
        "Alan M. Turing",
        "J. B. Kruskal",
        "Jon M. Kleinberg",
        "Éva Tardos",
        "Daniel R. Figueiredo"
    };

    auto dijkstraVertex = graph->getVertexByLabel("Edsger W. Dijkstra");
    auto dijkstraTree = graph->getDijkstraTree(dijkstraVertex, true);

    for (const auto& nome : pesquisadores) {
        auto pesquisadorVertex = graph->getVertexByLabel(nome);
        auto dist = graph->getDijkstraMinDistance(dijkstraTree, pesquisadorVertex);

        if (dist == std::numeric_limits<double>::max()) {
            std::cout << "Não há caminho entre Edsger W. Dijkstra e " << nome << std::endl;
        } else {
            std::cout << "Distancia entre Edsger W. Dijkstra e " << nome << " é " << dist << std::endl;
            std::cout << "Caminho entre Edsger W. Dijkstra e " << nome << " é: ";
            auto deque = graph->getDijkstraMinPathLabels("Edsger W. Dijkstra", nome, dijkstraTree);
            while (!deque.empty()) {
                std::cout << deque.back();
                deque.pop_back();
                if (!deque.empty()) {
                    std::cout << " -> ";
                }
            }
            std::cout << std::endl;

        }
    }
}

void studyCaseCollaborationPathWithDijkstra() {
    // Define o caminho do log
    std::string log_source_folder = "../output/";
    std::string log_file_name = "log_collaboration_path.txt";
    std::string log_file_path = log_source_folder + log_file_name;
    
    // Abre o arquivo de log para escrita
    std::ofstream log_file(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo de log em " << log_file_path << std::endl;
        return;
    }

    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "rede_colaboracao.txt";
    std::string graph_names_file_name = "rede_colaboracao_vertices.txt";
    std::string file_path = graph_source_folder + graph_file_name;
    std::string name_path = graph_source_folder + graph_names_file_name;

    Graph *graph = new Graph(file_path, GraphStructure::AdjVector, false, true, name_path);
    std::cout << "Graph Initialized" << std::endl;
    log_file << "Graph Initialized" << std::endl;

    std::vector<std::string> pesquisadores = {
        "Alan M. Turing",
        "J. B. Kruskal",
        "Jon M. Kleinberg",
        "Éva Tardos",
        "Daniel R. Figueiredo"
    };

    auto dijkstraVertex = graph->getVertexByLabel("Edsger W. Dijkstra");
    auto dijkstraTree = graph->getDijkstraTree(dijkstraVertex, true);

    for (const auto& nome : pesquisadores) {
        auto pesquisadorVertex = graph->getVertexByLabel(nome);
        auto dist = graph->getDijkstraMinDistance(dijkstraTree, pesquisadorVertex);

        if (dist == std::numeric_limits<double>::max()) {
            std::cout << "Não há caminho entre Edsger W. Dijkstra e " << nome << std::endl;
            log_file << "Não há caminho entre Edsger W. Dijkstra e " << nome << std::endl;
        } else {
            std::cout << "Distancia entre Edsger W. Dijkstra e " << nome << " é " << dist << std::endl;
            log_file << "Distancia entre Edsger W. Dijkstra e " << nome << " é " << dist << std::endl;

            std::cout << "Caminho entre Edsger W. Dijkstra e " << nome << " é: ";
            log_file << "Caminho entre Edsger W. Dijkstra e " << nome << " é: ";

            auto deque = graph->getDijkstraMinPathLabels("Edsger W. Dijkstra", nome, dijkstraTree);
            while (!deque.empty()) {
                std::cout << deque.back();
                log_file << deque.back();
                deque.pop_back();
                if (!deque.empty()) {
                    std::cout << " -> ";
                    log_file << " -> ";
                }
            }
            std::cout << std::endl;
            log_file << std::endl;
        }
    }

    // Fecha o arquivo de log
    log_file.close();
}


void testCaseResidualGraphInitializer() {
    // Define o caminho do log
    std::string log_source_folder = "../output/";
    std::string log_file_name = "log_graph_test.txt";
    std::string log_file_path = log_source_folder + log_file_name;
    
    // Abre o arquivo de log para escrita
    std::ofstream log_file(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo de log em " << log_file_path << std::endl;
        return;
    }

    // Caminho do arquivo do grafo
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_" + std::to_string(1) + ".txt";
    std::string file_path = graph_source_folder + graph_file_name;

    // Supondo que FlowGraph já tenha sido definido e está sendo utilizado aqui
    FlowGraph* graph = new FlowGraph(file_path, true, true);

    // Inicializa o mapa de arestas residuais
    std::unordered_map<size_t, FlowGraph::ResidualEdge> ResidualEdgeMap;
    graph->initResidualStructure(ResidualEdgeMap);

    // Imprime os elementos no console e no arquivo de log
    for (const auto& entry : ResidualEdgeMap) {
        const FlowGraph::ResidualEdge& edge = entry.second;

        // Imprimindo no console
        std::cout << "ID: " << edge.id 
                  << ", OriginalEdge: " << edge.originalEdge 
                  << ", Origin: " << edge.origin 
                  << ", Destiny: " << edge.destiny 
                  << ", Capacity: " << edge.capacity << std::endl;

        // Escrevendo no arquivo de log
        log_file << "ID: " << edge.id
                 << ", OriginalEdge: " << edge.originalEdge
                 << ", Origin: " << edge.origin
                 << ", Destiny: " << edge.destiny
                 << ", Capacity: " << edge.capacity << std::endl;
    }

    // Fecha o arquivo de log
    log_file.close();
}

// Supondo que FlowGraph e ResidualEdge estejam corretamente definidos
/*
void printAugmentingPath(int source, int target, int minCapacity) {
    // Define o caminho do log
    std::string log_source_folder = "../output/";
    std::string log_file_name = "log_graph_test.txt";
    std::string log_file_path = log_source_folder + log_file_name;
    
    // Abre o arquivo de log para escrita
    std::ofstream log_file(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo de log em " << log_file_path << std::endl;
        return;
    }

    // Caminho do arquivo do grafo
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_" + std::to_string(1) + ".txt";
    std::string file_path = graph_source_folder + graph_file_name;

    // Supondo que FlowGraph já tenha sido definido e está sendo utilizado aqui
    FlowGraph* graph = new FlowGraph(file_path, true, true);
    std::unordered_map<size_t, FlowGraph::ResidualEdge> ResidualEdgeMap;
    auto residualStructure = graph->initResidualStructure(ResidualEdgeMap);
    std::cout << "Grafo criado." << std::endl;
    
    // Chama o método findAugmentingPath para obter o caminho de aumento
    std::stack<FlowGraph::ResidualEdge> augmentingPath = graph->findAugmentingPath(source, target, minCapacity, ResidualEdgeMap, residualStructure);

    // Verifica se o caminho de aumento foi encontrado
    if (augmentingPath.empty()) {
        std::cout << "Nenhum caminho de aumento encontrado." << std::endl;
        log_file << "Nenhum caminho de aumento encontrado." << std::endl;
    } else {
        std::cout << "Caminho de aumento encontrado: " << std::endl;
        log_file << "Caminho de aumento encontrado: " << std::endl;

        // Itera sobre o stack e imprime as arestas do caminho
        while (!augmentingPath.empty()) {
            const FlowGraph::ResidualEdge& edge = augmentingPath.top();
            augmentingPath.pop();

            // Imprimindo no console
            std::cout << "ID: " << edge.id 
                      << ", OriginalEdge: " << edge.originalEdge 
                      << ", Origin: " << edge.origin 
                      << ", Destiny: " << edge.destiny 
                      << ", Capacity: " << edge.capacity << std::endl;

            // Escrevendo no arquivo de log
            log_file << "ID: " << edge.id
                     << ", OriginalEdge: " << edge.originalEdge
                     << ", Origin: " << edge.origin
                     << ", Destiny: " << edge.destiny
                     << ", Capacity: " << edge.capacity << std::endl;
        }
    }

    // Fecha o arquivo de log
    log_file.close();
}

*/

void testCaseCalculateMaxFlow(const int source, const int target, const int graph_number) {
    // Define o caminho do log
    std::string log_source_folder = "../output/";
    std::string log_file_name = "log_graph_test.txt";
    std::string log_file_path = log_source_folder + log_file_name;
    
    // Abre o arquivo de log para escrita
    std::ofstream log_file(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo de log em " << log_file_path << std::endl;
        return;
    }

    // Caminho do arquivo do grafo
    std::string graph_source_folder = "../test_subjects/";
    std::string graph_file_name = "grafo_rf_" + std::to_string(graph_number) + ".txt";
    std::string file_path = graph_source_folder + graph_file_name;

        // Supondo que FlowGraph já tenha sido definido e está sendo utilizado aqui
    FlowGraph* graph = new FlowGraph(file_path, true, true);

    // Captura o tempo antes de calcular o fluxo máximo
    auto start = std::chrono::high_resolution_clock::now();

    // Calcula o fluxo máximo
    int maxFlow = graph->calculateMaxFlow(source, target);

    // Captura o tempo após calcular o fluxo máximo
    auto end = std::chrono::high_resolution_clock::now();

    // Calcula a duração do cálculo
    std::chrono::duration<double> duration = end - start;

    // Imprime o tempo de execução
    std::cout << "Fluxo máximo no grafo é: " << maxFlow << std::endl;
    std::cout << "Tempo de execução do cálculo de fluxo máximo: " 
              << duration.count() << " segundos." << std::endl;

    // Registra o resultado no arquivo de log
    log_file << "Fluxo máximo no grafo é: " << maxFlow << std::endl;
    log_file << "Tempo de execução do cálculo de fluxo máximo: " 
             << duration.count() << " segundos." << std::endl;

    // Fecha o arquivo de log
    log_file.close();
}

int main(){
    //std::cout << "starting tests" << std::endl;
    for (int i = 1; i <= 6; i++) {
    testCaseCalculateMaxFlow(1,2, i);
    }
    return 0;
}

/*
Lendo grafo do arquivo: grafo_rf_1.txt
Fluxo máximo entre 1 e 2 no grafo grafo_rf_1.txt: 1058
Lendo grafo do arquivo: grafo_rf_2.txt
Fluxo máximo entre 1 e 2 no grafo grafo_rf_2.txt: 11189
Lendo grafo do arquivo: grafo_rf_3.txt
Fluxo máximo entre 1 e 2 no grafo grafo_rf_3.txt: 2964
*/