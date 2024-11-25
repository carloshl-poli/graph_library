/*
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
    AdjVector residualStructure = AdjVector(this->vertexAmount, true);
    for (const auto& U : this->structure->body) {
        for (const auto& W : U) {
            size_t id_UV = residualStructure.insertEdge(W.origin, W.to, W.weight, true);
            auto residualEdge_UV = ResidualEdge(getEdgeKey(W.origin,W.to), id_UV, W.origin, W.to, W.weight);
            ResidualEdgeMap[id_UV] = residualEdge_UV;

            size_t id_VU = residualStructure.insertEdge(W.to, W.origin, W.weight, true);
            auto residualEdge_VU = ResidualEdge(getEdgeKey(W.to, W.origin), id_VU, W.to, W.origin, 0);
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

            residualEdgeMap[idForwardEdge].sisterID = idReverseEdge;
            residualEdgeMap[idReverseEdge].sisterID = idForwardEdge;
        }
    }

}



std::stack<FlowGraph::ResidualEdge> FlowGraph::findAugmentingPath(int source,
    int target, int minCapacity, std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap, AdjVector residualStructure, int &bottleneck) {
        std::unordered_map<int, FlowGraph::ResidualEdge> edgeTree;
        std::stack<int> S;
        std::unordered_map<int, int> mark(this->vertexAmount);
        for (int i = 1; i <= vertexAmount; i++){
            mark[i] = 0;
        }
        bool targetFound = false;
        S.push(source);
        while (!S.empty() || !targetFound){
            int V = S.top();
            S.pop();
            if (mark[V] == 0){
                mark[V] = 1;
                for (const auto& node: residualStructure.getAdjNodeArray(V)){
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
            bottleneck = std::numeric_limits<int>::infinity();
            int currentOrigin = target;
            while (currentOrigin != source) {
                path.push(edgeTree[currentOrigin]);
                if (bottleneck > path.top().capacity) bottleneck = path.top().capacity;
                currentOrigin = edgeTree[currentOrigin].origin;
            }
        }
        return path;

}

void FlowGraph::Augment(std::stack<FlowGraph::ResidualEdge> path, std::unordered_map<std::string, FlowGraph::FlowEdge> &flowEdgeMap,
 int bottleneck, std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap) {

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
            ResidualEdgeMap[edge.id].capacity =  flowEdgeMap[edge.originalEdge].flow;
        }
    }

}

std::unordered_map<std::string, FlowGraph::FlowEdge> FlowGraph::FordFulkerson(int source, int target)
{
    // Mapeia o ID da aresta residual para sua estrutura ResidualEdge
    std::unordered_map<size_t, ResidualEdge> residualEdgeMap;

    // Cria uma estrutura de adjacência para armazenar as arestas residuais
    AdjVector residualStructure = AdjVector(this->vertexAmount, true);

    // Mapeia as arestas originais aos seus dados de fluxo, capacidade, ...
    std::unordered_map<std::string, FlowGraph::FlowEdge> flowEdgeMap;

    helper_FordFulkerson(residualEdgeMap, residualStructure, flowEdgeMap);

    int sourceMaxFlow = this->structure->getAdjNodeArray(source).size();
    int currentMinCapacity = bitFloor(sourceMaxFlow);

    while (currentMinCapacity > 0) {
        int bottleneck;
        auto path = findAugmentingPath(source, target, currentMinCapacity, residualEdgeMap, residualStructure, bottleneck);
        if (path.empty()) {
            if (currentMinCapacity == 1) break;
            else currentMinCapacity = currentMinCapacity / 2;
        }
        else {
        Augment(path, flowEdgeMap, bottleneck, residualEdgeMap);
        }
    }

    return flowEdgeMap;
    
}

int FlowGraph::calculateMaxFlow(const std::unordered_map<std::string, FlowEdge> &flowEdgeMap, const int source) {
    int maxFlow = 0;
    std::string edgeString;
    auto sourceDivergentEdges = this->structure->getAdjNodeArray(source);
    for (const auto& edge : sourceDivergentEdges) {
        edgeString = getEdgeKey(source, edge.to);
        maxFlow += flowEdgeMap.at(edgeString).flow;
    }

    return maxFlow;
}

int FlowGraph::calculateMaxFlow(const int source, const int target) {
    auto flowEdgeMap = FordFulkerson(source, target);
    return calculateMaxFlow(flowEdgeMap, source);
}
*/