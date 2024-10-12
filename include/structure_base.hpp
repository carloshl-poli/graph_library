#ifndef STRUCTURE_BASE_HPP
#define STRUCTURE_BASE_HPP

#include <iostream>
#include <vector>
#include <variant>
#include <utility>
#include <optional>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <optional>

using IntVec = std::vector<int>;
using PairVec = std::vector<std::pair<int, std::optional<double>>>;
using ReturnType = std::variant<IntVec, PairVec>;

class Structure {
protected:
    int vertexAmount;
    int edgeAmount;
    std::vector<int> degreeVec;
    std::vector<double> weightVec;
    
public:
    bool hasWeight;

    //Structure::Structure();
    //Structure(const std::string &path, bool isDirected);

    virtual ~Structure() = default;
    virtual bool hasEdgeUV(int V, int U) = 0;
    virtual void resize(int size) = 0;
    virtual void insertEdge(int u,int V,double w) = 0;
    virtual std::optional<double> getWeightUV(int V, int U) = 0;
    virtual void printGraph() = 0;
    virtual ReturnType getUAdjArray(int U, bool getWeight = false) = 0;

    int getVertexAmount();
    int getEdgeAmount();
    int getUDegree(int U);
    double getUWeight(int U);
    
};


#endif