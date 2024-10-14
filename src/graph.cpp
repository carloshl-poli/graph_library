#include "graph.hpp"




// Construtor
Graph::Graph(std::string &path, GraphStructure structure, bool isDirected){
    switch (structure) {
        case GraphStructure::AdjMatrix:
            this->structure = std::make_unique<AdjMatrix>(path, isDirected);
            break;
        case GraphStructure::AdjVector:
            this->structure = std::make_unique<AdjVector>(path, isDirected);
            break;
        default:
            throw std::out_of_range("Error: Invalid or not supported Structure");
            break;
    }

    this->vertexAmount = this->structure->getVertexAmount();
    this->edgeAmount = this->structure->getEdgeAmount();
    this->hasWeight = this->structure->hasWeight;

}

double Graph::getMean(){
    auto structData = this->structure->getDataArray();
    if (structData.empty()) return 0.0;

    double degreeSum = 0.0;
    for (size_t v = 0; v < structData.size(); v++){
        degreeSum += structData[v].degree;
    }
    return degreeSum / structData.size();
}

/// @note Ready for tests
double Graph::getMedian() {
    auto structData = this->structure->getDataArray();
    int n = structData.size(); 
        if (n % 2 == 0){

            int medianIndex = n / 2;
            return (structData[medianIndex].degree + structData[medianIndex + 1].degree ) / 2;
        }
        else{
            int medianIndex = std::floor(n / 2);
            return structData[medianIndex].degree; 
        }
}

int Graph::getMinDegree() {
    int minDegree = std::numeric_limits<int>::max();
    auto structData = this->structure->getDataArray();

    for (const auto& pair : structData){
        const auto& data = pair.second;
        if (data.degree < minDegree){
            minDegree = data.degree;
        }
    }    
    return minDegree;
}

int Graph::getMaxDegree(){
    int maxDegree = std::numeric_limits<int>::min();
    auto structData = this->structure->getDataArray();

    for (const auto& pair : structData){
        const auto& data = pair.second;
        if (data.degree > maxDegree){
            maxDegree = data.degree;
        }
    }    
    return maxDegree;
}
/// @note Ready for tests 
Graph::ReturnGraphDataMap Graph::getBFSTree(int U) {
    std::unordered_map<int, Graph::Data> bfsTree;
    std::unordered_map<int, int> mark(this->vertexAmount);
    std::queue<int> S;
    for (int I = 1; I <= this->vertexAmount; I++){
        mark[I] = 0;
    }
    bfsTree[U].parent = -1;
    bfsTree[U].level = 0;
    mark[U]= 1;
    S.push(U);
    
    while (!S.empty()){
        int V = S.front();
        S.pop();
        for (const int& W : this->structure->getAdjArray(V)){
            if (mark[W] == 0){
                mark[W] = 1;
                bfsTree[W].level = bfsTree[V].level + 1;
                bfsTree[W].parent = V;
                S.push(W);
            }
        }
    }
    return bfsTree;
}
/// @note Ready for tests
Graph::ReturnGraphDataMap Graph::getDFSTree(int U) {
    std::unordered_map<int, Graph::Data> dfsTree;
    std::stack<int> P;
    std::unordered_map<int, int> mark(this->vertexAmount);
    for (int i = 1; i <= vertexAmount; i++){
        mark[i] = 0;
    }
    dfsTree[U].level = 0;
    dfsTree[U].parent = -1;
    P.push(U);
    while (!P.empty()){
        int V = P.top();
        if (mark[V] == 0){
            mark[V] = 1;
            for (const int& W: this->structure->getAdjArray(V)){
                if (mark[W] == 0){
                    dfsTree[W].level = dfsTree[V].level + 1;
                    dfsTree[W].parent = V;
                }
                P.push(W);
            }

        }
    }

    return dfsTree;
}

/// @brief Calculate the Minimal distance between 2 vertices
/// @param U is the first vertice
/// @param V is the second vertice
/// @return the distance between U and V or nullopt case there is no path between them
/// @note Ready for tests
std::optional<int> Graph::getUVDistance(int U, int V) {
    std::unordered_map<int, Data> bfsTreeU = this->getBFSTree(U);
    try{
        return bfsTreeU.at(V).level;
    }
    catch (const std::out_of_range& e){
        return std::nullopt;
    }   
}

int Graph::getExactDiameter(){
    int currentDiameter = 0;
    int markReference = 0;
    std::unordered_map<int, int> mark;
    for (int V = 1; V <= vertexAmount; V++){
        mark[V] = 0;
    }
    for (int U = 1; U <= vertexAmount; U++){
        markReference++;
        std::unordered_map<int, int> level;
        std::queue<int> S;
        level[U] = 0;
        mark[U] = markReference;
        S.push(U);
        while (!S.empty()){
            int V = S.front();
            S.pop();
            for (const int& W : this->structure->getAdjArray(V)){
                if (mark[W] != markReference){
                    mark[W] = markReference;
                    level[W] = level[V] + 1;
                    if (level[W] > currentDiameter){
                        currentDiameter = level[W];
                    }
                    S.push(W);
                }
            }
        }
    }
    return currentDiameter;
}


int Graph::getApproxDiameter(){
    

}
