#include "adj_vector.hpp"

/*
std::string getEdgeKey(int U, int V) {
    // Determina o menor e o maior número
    int menor = (U < V) ? U : V;
    int maior = (U > V) ? U : V;

    // Concatenando as strings diretamente
    return std::to_string(menor) + " " + std::to_string(maior);
}
*/
AdjVector::AdjVector(size_t size, bool isWeighted) {
    resize(size);
    this->isWeighted = isWeighted;
}

AdjVector::AdjVector(const std::string &path, bool isWeighted, bool isDirected)
{
    initStruct(path, isDirected, isWeighted);
}

void AdjVector::insertEdge(int U, int V, double weight) {
    id_counter++;
    auto node = NodeBase(U, weight, V, id_counter);
    this->body[U-1].emplace_back(node);
}

size_t AdjVector::insertEdge(int U, int V, double W, bool getID) {
    insertEdge(U, V, W);
    return id_counter;
}

void AdjVector::resize(int size){
    this->body.resize(size);
}

int AdjVector::getEdgeUV(int U, int V){
    int u = U - 1;
    for (int v = 0; v < this->body[u].size(); v++){
        if (this->body[u][v].to == V){
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
    if (!isWeighted){
        throw std::logic_error("This Graph is weightless");
    }
    else if (this->hasEdgeUV(U, V)){
        int v = this->getEdgeUV(U,V);
        return this->body[U-1][v].weight;
    }
    else {
        throw std::invalid_argument("Error: There is no edge between vertices "
            + std::to_string(V) + " and " + std::to_string(U) + ".");
    }
}

std::vector<int> AdjVector::getAdjArray(int U) {
    int u = U - 1;
    std::vector<int> intArray;
    for (int v = 0; v < this->body[u].size(); v++){
        intArray.push_back(this->body[u][v].to);
    }
    return intArray;
}

std::vector<std::pair<int, double>> AdjVector::getAdjWeightedArray(int U) {
    if (!isWeighted){
        throw std::logic_error("This Graph is weightless");
    }
    int u = U - 1;
    std::vector<std::pair<int, double>> weightArray;
    for (int v = 0; v < this->body[u].size(); v++){
        weightArray.push_back(std::make_pair(this->body[u][v].to, this->body[u][v].weight));
    }
    return weightArray;
}

std::string AdjVector::getType() {
    return "AdjVector";
}

std::vector<AdjVector::NodeBase> AdjVector::getAdjNodeArray(int U) {
    int u = U - 1;
    return this->body[u];
}