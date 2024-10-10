#include "adj_vector.hpp"

void AdjVector::insertEdge(int u, int V, double weight) {
    this->body[u].emplace_back(V, weight);
}

void AdjVector::resize(int size){
    this->body.resize(size);
}

int AdjVector::getEdgeUV(int U, int V){
    for (int index = 0; index < this->body[U - 1].size(); index++){
        if (this->body[U-1][index].first == V){
            return index;
        }
    }
    throw std::out_of_range("Edge UV not inside graph");
}

bool AdjVector::hasEdgeUV(int U, int V){
    int index;
    try
    {
        index = this->getEdgeUV(U,V);
        return true;
    }
    catch(const std::out_of_range& e)
    {
        return false;
    }
}
    

double AdjVector::getWeightUV(int U, int V){
    int v = this->getEdgeUV(U,V);
    return this->body[U-1][v].second;
}

void AdjVector::setWeightUV(int U, int V, double newWeight){
    int v = this->getEdgeUV(U,V);
    this->body[U-1][v].second = newWeight;
}

/// @todo implement this 
void AdjVector::printGraph(){

}

