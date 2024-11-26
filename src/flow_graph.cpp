



#include "flow_graph.hpp"


int bitFloor(int x) {
    if (x <= 0)
        return 0;

    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);

    return x - (x >> 1); // Remove o bit extra para retornar a maior potência <= x
}


std::string getEdgeKey(int U, int V) {
    // Determina o menor e o maior número
    int menor = (U < V) ? U : V;
    int maior = (U > V) ? U : V;

    // Concatenando as strings diretamente
    return std::to_string(menor) + " " + std::to_string(maior);
}


FlowGraph::FlowGraph(std::string &path, bool isDirected, bool isWeighted) {
    this->structure = std::make_unique<AdjVector>(path, isWeighted, isDirected);
    this->vertexAmount = this->structure->getVertexAmount();
    this->edgeAmount = this->structure->getEdgeAmount();
    this->hasWeight = this->structure->hasWeight();
}

AdjVector FlowGraph::initResidualStructure(std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap) {
    
    
    // Cria a estrutura residual
    AdjVector residualStructure = AdjVector(this->vertexAmount, true);

    // Itera sobre a estrutura do grafo original
    for (const auto& U : this->structure->body) {

        for (const auto& W : U) {

            // Insere a aresta original na estrutura residual
            size_t id_UV = residualStructure.insertEdge(W.origin, W.to, W.weight, true);

            // Cria a aresta residual UV
            auto residualEdge_UV = ResidualEdge(getEdgeKey(W.origin, W.to), id_UV, W.origin, W.to, W.weight);

            // Armazena no mapa
            ResidualEdgeMap[id_UV] = residualEdge_UV;

            // Insere a aresta reversa na estrutura residual com capacidade 0
            size_t id_VU = residualStructure.insertEdge(W.to, W.origin, W.weight, true);

            // Cria a aresta residual VU
            auto residualEdge_VU = ResidualEdge(getEdgeKey(W.to, W.origin), id_VU, W.to, W.origin, 0);

            // Armazena a aresta reversa no mapa
            ResidualEdgeMap[id_VU] = residualEdge_VU;
        }
    }

    return residualStructure;
}


void FlowGraph::helper_FordFulkerson(
    std::unordered_map<size_t, ResidualEdge> &residualEdgeMap,
    AdjVector &residualStructure,
    std::unordered_map<std::string, FlowGraph::FlowEdge> &flowEdgeMap
) {

    // Itera pelas arestas do grafo original
    for (const auto& edgeList : this->structure->body) {

        for (const auto& edge : edgeList) {

            // Adiciona a aresta (U -> V) ao grafo residual
            auto edge_UV = getEdgeKey(edge.origin, edge.to);
            size_t idForwardEdge = residualStructure.insertEdge(
                edge.origin, edge.to, edge.weight, true
            );

            residualEdgeMap[idForwardEdge] = ResidualEdge(
                edge_UV, idForwardEdge,
                edge.origin, edge.to, edge.weight
            );

            flowEdgeMap[edge_UV] = FlowEdge(edge_UV, edge.weight, 0);

            // Adiciona a aresta reversa (V -> U) ao grafo residual
            size_t idReverseEdge = residualStructure.insertEdge(
                edge.to, edge.origin, 0, true
            );

            residualEdgeMap[idReverseEdge] = ResidualEdge(
                getEdgeKey(edge.to, edge.origin), idReverseEdge,
                edge.to, edge.origin, 0
            );

            // Definir as arestas irmãs para cada aresta residual
            residualEdgeMap[idForwardEdge].sisterID = idReverseEdge;
            residualEdgeMap[idReverseEdge].sisterID = idForwardEdge;
        }
    }

}




std::stack<FlowGraph::ResidualEdge> FlowGraph::findAugmentingPath(
    int source, int target, int minCapacity, 
    std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap, 
    AdjVector residualStructure, int &bottleneck) {

    std::unordered_map<int, FlowGraph::ResidualEdge> edgeTree;
    std::stack<int> S;
    std::unordered_map<int, int> mark(this->vertexAmount);

    // Inicializa o marcador para todos os vértices
    for (int i = 1; i <= vertexAmount; i++) {
        mark[i] = 0;
    }

    bool targetFound = false;
    S.push(source);

    // Busca em profundidade até encontrar o target ou explorar todos os vértices
    while (!S.empty() && !targetFound) {
        int V = S.top();
        S.pop();

        if (mark[V] == 0) {
            mark[V] = 1;

            for (const auto& node : residualStructure.getAdjNodeArray(V)) {
                if (mark[node.to] == 0 && ResidualEdgeMap[node.id].capacity >= minCapacity) {
                    
                    edgeTree[node.to] = ResidualEdgeMap[node.id];

                    if (node.to == target) {
                        targetFound = true;
                        break;
                    }
                    S.push(node.to);
                }
            }
        }
    }

    std::stack<FlowGraph::ResidualEdge> path;
    if (targetFound) {
        
        int currentOrigin = target;
        bottleneck = std::numeric_limits<int>::max();
        while (currentOrigin != source) {
            path.push(edgeTree[currentOrigin]);
            //std::cout << "Capacidade da aresta:" << path.top().origin << " " << path.top().destiny << " é: " << path.top().capacity << std::endl;
            if (bottleneck > path.top().capacity) {
                bottleneck = path.top().capacity;
            }
            currentOrigin = edgeTree[currentOrigin].origin;
        }
    } else {
        //std::cout << "Caminho de aumento não encontrado entre " << source << " e " << target << "." << std::endl;
    }

    return path;
}


void FlowGraph::Augment(
    std::stack<FlowGraph::ResidualEdge> path,
    std::unordered_map<std::string, FlowGraph::FlowEdge> &flowEdgeMap,
    int bottleneck, 
    std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap) {

    while (!path.empty()) {
        auto edge = path.top();
        path.pop();
        auto edgeString = getEdgeKey(edge.origin, edge.destiny);

        if (edge.originalEdge == edgeString) {

            flowEdgeMap[edge.originalEdge].flow += bottleneck;

            ResidualEdgeMap[edge.id].capacity = flowEdgeMap[edge.originalEdge].capacity - flowEdgeMap[edge.originalEdge].flow;

            ResidualEdgeMap[edge.sisterID].capacity = flowEdgeMap[edge.originalEdge].flow;
        }
        else {

            flowEdgeMap[edge.originalEdge].flow -= bottleneck;

            ResidualEdgeMap[edge.sisterID].capacity = flowEdgeMap[edge.originalEdge].capacity - flowEdgeMap[edge.originalEdge].flow;

            ResidualEdgeMap[edge.id].capacity = flowEdgeMap[edge.originalEdge].flow;
        }
    }

    //std::cout << "Operação de aumento concluída." << std::endl;
}


std::unordered_map<std::string, FlowGraph::FlowEdge> FlowGraph::FordFulkerson(int source, int target)
{
    //std::cout << "Iniciando o algoritmo Ford-Fulkerson." << std::endl;
    
    // Mapeia o ID da aresta residual para sua estrutura ResidualEdge
    std::unordered_map<size_t, ResidualEdge> residualEdgeMap;

    // Cria uma estrutura de adjacência para armazenar as arestas residuais
    AdjVector residualStructure = AdjVector(this->vertexAmount, true);
    
    // Mapeia as arestas originais aos seus dados de fluxo, capacidade, ...
    std::unordered_map<std::string, FlowGraph::FlowEdge> flowEdgeMap;

    // Chama o helper para inicializar as estruturas residuais
    //std::cout << "Inicializando a estrutura residual..." << std::endl;
    helper_FordFulkerson(residualEdgeMap, residualStructure, flowEdgeMap);
    
    // Obtém o número máximo de fluxos que podem ser enviados do source
    int sourceMaxFlow = 0;
    for (const auto edge : this->structure->getAdjNodeArray(source)) {
        if (std::fmod(edge.weight, 1.0) != 0 ) throw std::logic_error("Error! no decimal weight are supported");
        sourceMaxFlow += static_cast<int>(edge.weight);
    }
    sourceMaxFlow = sourceMaxFlow == 0 ? 1 : sourceMaxFlow;
    int currentMinCapacity = bitFloor(sourceMaxFlow);
    //std::cout << "Fluxo máximo inicial estimado para o nó fonte " << source << ": " << sourceMaxFlow << std::endl;
    //std::cout << "Capacidade mínima inicial: " << currentMinCapacity << std::endl;

    // Loop principal do algoritmo Ford-Fulkerson
    while (currentMinCapacity > 0) {
        int bottleneck;
        
        // Encontra o caminho aumentante no grafo residual
        //std::cout << "Procurando caminho aumentante com capacidade mínima " << currentMinCapacity << std::endl;
        auto path = findAugmentingPath(source, target, currentMinCapacity, residualEdgeMap, residualStructure, bottleneck);

        if (path.empty()) {
            //std::cout << "Nenhum caminho aumentante encontrado." << std::endl;
            if (currentMinCapacity == 1) {
                //std::cout << "Capacidade mínima chegou a 1, encerrando o algoritmo." << std::endl;
                break;
            } else {
                currentMinCapacity = currentMinCapacity / 2;
                //std::cout << "Diminuição da capacidade mínima para: " << currentMinCapacity << std::endl;
            }
        } else {
            //std::cout << "Caminho aumentante encontrado com gargalo: " << bottleneck << std::endl;
            // Aumenta o fluxo ao longo do caminho encontrado
            Augment(path, flowEdgeMap, bottleneck, residualEdgeMap);
        }
    }

    //std::cout << "Algoritmo Ford-Fulkerson concluído." << std::endl;
    
    // Retorna o mapa de arestas de fluxo, contendo os fluxos finais
    return flowEdgeMap;
}


int FlowGraph::calculateMaxFlow(const std::unordered_map<std::string, FlowEdge> &flowEdgeMap, const int source) {
    //std::cout << "Calculando o fluxo máximo a partir do nó fonte " << source << std::endl;
    
    int maxFlow = 0;
    std::string edgeString;
    
    // Obtém as arestas divergentes do nó fonte
    auto sourceDivergentEdges = this->structure->getAdjNodeArray(source);
    //std::cout << "Número de arestas divergentes do nó fonte: " << sourceDivergentEdges.size() << std::endl;
    
    for (const auto& edge : sourceDivergentEdges) {
        edgeString = getEdgeKey(source, edge.to);
        
        // Adiciona o fluxo da aresta ao fluxo total
        //std::cout << "Aresta: " << edgeString << " - Fluxo: " << flowEdgeMap.at(edgeString).flow << std::endl;
        maxFlow += flowEdgeMap.at(edgeString).flow;
    }

    //std::cout << "Fluxo máximo calculado: " << maxFlow << std::endl;
    return maxFlow;
}

int FlowGraph::calculateMaxFlow(const int source, const int target) {
    //std::cout << "Calculando o fluxo máximo entre os nós " << source << " e " << target << std::endl;
    
    // Executa o algoritmo Ford-Fulkerson para calcular os fluxos
    auto flowEdgeMap = FordFulkerson(source, target);
    
    // Calcula o fluxo máximo com base nas arestas do fluxo
    int maxFlow = calculateMaxFlow(flowEdgeMap, source);
    //std::cout << "Fluxo máximo entre " << source << " e " << target << " é: " << maxFlow << std::endl;
    
    return maxFlow;
}


