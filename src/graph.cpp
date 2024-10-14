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

/// @todo Rewrite using new method getWeightedDegreeVec 
double Graph::getMean(bool weighted){
    std::vector<int> degreeVec = this->structure->getDegreeVec();
    if (degreeVec.empty()) return 0.0;

    if (weighted) {
        std::vector<std::optional<double>> weightVec = this->structure->getWeightVec();
        double weightedSum = 0.0;
        double weightSum = 0.0;

        for (size_t i = 0; i < degreeVec.size(); i++){
            if (weightVec[i]){
                weightedSum += degreeVec[i] * *weightVec[i];
                weightSum += *weightVec[i];
            }
        }
        return weightSum == 0.0 ? 0.0 : weightedSum/weightSum;
    }
    else {
        double degreeSum = 0.0;
        for (size_t i = 0; i < degreeVec.size(); i++){
            degreeSum += degreeVec[i];
        }
        return degreeSum / degreeVec.size();

    }
    
}

/// @warning incomplete method DO NOT USE
double Graph::getMedian(bool weighted) {
    std::vector<int> degreeVec = this->structure->getDegreeVec();
    int n = degreeVec.size();
    if (!weighted){
        if (n % 2 == 0){

        }

    }
    std::vector<std::optional<double>> weightVec = this->structure->getWeightVec();
    
    

}

std::optional<double> Graph::getMinDegree(bool weighted) {
    if (!weighted){
        std::vector<int> degreeVec = this->structure->getDegreeVec();
        return *std::min_element(degreeVec.begin(), degreeVec.end());

    }
    std::vector<std::optional<double>> weightedVec = this->structure->getWeightedDegreeVec();
    return *std::min_element(weightedVec.begin(), weightedVec.end());
    
}

std::optional<double> Graph::getMaxDegree(bool weighted){
    if (!weighted){
        std::vector<int> degreeVec = this->structure->getDegreeVec();
        return *std::max_element(degreeVec.begin(), degreeVec.end());

    }
    std::vector<std::optional<double>> weightedVec = this->structure->getWeightedDegreeVec();
    return *std::max_element(weightedVec.begin(), weightedVec.end());
    
}

std::unordered_map<int, Data> Graph::getBFSTree(int U) {
    std::unordered_map<int, Data> bfsTree;
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

std::unordered_map<int, std::pair<int, int>> Graph::getDFSTree(int U) {
    using s = Structure;
    std::unordered_map<int, std::pair<int, int>> dfsTree;
    std::stack<int> P;
    std::vector<int> levelVec(this->vertexAmount);
    std::vector<int> parentVec(this->vertexAmount);
    std::vector<int> markVec(this->vertexAmount, 0);
    P.push(U);
    s::setUData(U, levelVec, 0);
    s::setUData(U, parentVec, NULL);
    while (!P.empty()){
        int V = P.top();
        int levelV = s::getUData(V, levelVec);
        if (s::getUData(V, markVec) == 0){
            s::setUData(V, markVec, 1);
            for (const int& W: this->structure->getAdjArray(V)){
                if (s::getUData(W, markVec) == 0){
                    s::setUData(W, levelVec, levelV + 1);
                    s::setUData(W, parentVec, V);
                    dfsTree[W] = std::make_pair(levelV + 1, V);
                }
                P.push(W);
            }

        }
    }

    return dfsTree;
}

int Graph::getUVDistance(int U, int V) {
    std::unordered_map<int, Data> bfsTreeU = this->getBFSTree(U);
    return bfsTreeU[U].level;
}

int Graph::getGraphDiameter(bool getAccurate){
    using s = Structure;
    int currentDiameter = 0;
    if (getAccurate){

    }
    else{
        std::vector<int> markVec(this->vertexAmount, 0);
        for (int U = 1; U <= this->vertexAmount; U++){
            if (s::getUData(U, markVec) == 0){
                std::unordered_map<int, int> levelDict;
                std::stack<int> P;
                //s::setUData(U, markVec, 1);
                P.push(U);
                levelDict[U] = 0;
                while (!P.empty()){
                    int V = P.top();
                    P.pop();
                    if (s::getUData(V, markVec) == 0){
                        s::setUData(V, markVec, 1);
                        for (const int& W : this->structure->getAdjArray(V)){
                            if (s::getUData(W, markVec) == 0){
                                
                            }
                        }
                    }
                }
            }


        }
    }
}
