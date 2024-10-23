#include "adj_vector.hpp"



AdjVector::AdjVector(const std::string &path, bool isWeighted, bool isDirected) {
    this->initStruct(path, isDirected, isWeighted);
}

void AdjVector::insertEdge(int U, int V, double weight) {
    this->body[U-1].emplace_back(V, weight);
}

void AdjVector::resize(int size){
    this->body.resize(size);
}

int AdjVector::getEdgeUV(int U, int V){
    int u = U - 1;
    for (int v = 0; v < this->body[u].size(); v++){
        if (this->body[u][v].first == V){
            return v;
        }
    }
    throw std::invalid_argument("Error: There is no edge between vertices "
        + std::to_string(V) + " and " + std::to_string(U) + ".");
}

bool AdjVector::hasEdgeUV(int U, int V) {
    int index;
    try
    {
        index = this->getEdgeUV(U,V);
        return true;
    }
    catch(const std::invalid_argument& e)
    {
        return false;
    }
}
    

double AdjVector::getWeightUV(int U, int V){
    int v = this->getEdgeUV(U,V);
    return this->body[U-1][v].second;
}

std::vector<int> AdjVector::getAdjArray(int U) {
    int u = U - 1;
    std::vector<int> intArray;
    for (int v = 0; v < this->body[u].size(); v++){
        intArray.push_back(this->body[u][v].first);
    }
    return intArray;
}

std::vector<std::pair<int, double>> AdjVector::getAdjWeightedArray(int U) {
    if (!isWeighted){
        throw std::logic_error("This Graph is weightless");
    }
    int u = U - 1;
    return this->body[u];


    /*
    std::vector<std::pair<int, double>> weightedArray;
    for (const auto& pair : this->body[u]) {
            weightedArray.push_back({pair.first, pair.second});
    }
    */
}
