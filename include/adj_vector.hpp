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


#endif