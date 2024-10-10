#ifndef ADJ_VECTOR_HPP
#define ADJ_VECTOR_HPP

#include "structure_base.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>

class AdjVector : public Structure
{
private:
    std::vector<std::vector<std::pair<int, double>>> body;
public:
    AdjVector(const std::string &path, bool isDirected = false);
    ~AdjVector();
    int getEdgeUV(int U, int V);
    void setWeightUV(int U, int V, double newWeight);
    void printGraph() override;

    void resize(int size) override;
    void insertEdge(int u,int V,double w) override;
    bool hasEdgeUV(int U, int V) override;
    double getWeightUV(int U, int V) override;

    
};

AdjVector::AdjVector(const std::string &path, bool isDirected) {
    std::string line;
    std::ifstream graphFile(path);

    if (!graphFile){
        std::cerr << "Error! Couldn't open graph" << std::endl;
        exit(EXIT_FAILURE);
    }
    //Set vertex Amount to the first line
    if (getline(graphFile, line)){
        this->vertexAmount = std::stoi(line);
    }

    this->resize(this->vertexAmount);


    while (getline(graphFile, line)){
        std::istringstream iss(line);
        int u, v, U, V;
        double weight;

        if (iss >> U >> V){
            u = U - 1;
            v = V - 1;

            if (u < 0 || v < 0 || u >= this->vertexAmount || v >= this->vertexAmount) {
                std::cerr << "Error! índex out of bounds: " << U << ", " << V << std::endl;
                exit(EXIT_FAILURE);
            }

            if (!(iss >> weight)){
                weight = 1.0;
            } 
            this->insertEdge(u,V, weight);
            this->edgeAmount++;
            if (!isDirected){
                this->insertEdge(v,U, weight);
            }
        }

    }
    
}

AdjVector::~AdjVector()
{
}


#endif