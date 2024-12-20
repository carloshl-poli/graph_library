#include "adj_matrix.hpp"


AdjMatrix::AdjMatrix(const std::string &path, bool isWeighted, bool isDirected) {
    this->initStruct(path, isDirected, isWeighted);
}

void AdjMatrix::insertEdge(int U, int V, double w) {
    this->body[U-1][V-1] = w;
}

void AdjMatrix::resize(int n){
    this->body.resize(n, std::vector<std::optional<double>>(n, std::nullopt));
}

bool AdjMatrix::hasEdgeUV(int U, int V){
    return this->body[U-1][V-1].has_value();
}

double AdjMatrix::getWeightUV(int U, int V){
    if (!isWeighted){
        throw std::logic_error("This Graph is weightless");
    }
    else if (this->hasEdgeUV(U, V)){
        return this->body[U-1][V-1].value();
    }
    else {
        throw std::invalid_argument("Error: There is no edge between vertices "
            + std::to_string(V) + " and " + std::to_string(U) + ".");
    }
}

std::vector<int> AdjMatrix::getAdjArray(int U) {
    std::vector<int> adjArray;
    int u = U - 1;
    for (int v = 0; v < this->vertexAmount; v++){
        int V = v + 1;
        if (this->body[u][v].has_value()){
            adjArray.push_back(V);
        }
    }
    return adjArray;
}

std::vector<std::pair<int, double>> AdjMatrix::getAdjWeightedArray(int U) {
    if (!isWeighted){
        throw std::logic_error("This Graph is weightless");
    }

    std::vector<std::pair<int, double>> adjWeightedArray;
    int u = U - 1;
    for (int v = 0; v < this->vertexAmount; v++){
        int V = v + 1;
        if (this->body[u][v].has_value()){
            adjWeightedArray.push_back(std::pair<int, double>(V, this->body[u][v].value()));
        }
    }

    return adjWeightedArray;
}

std::string AdjMatrix::getType() {
    return "AdjMatrix";
}
