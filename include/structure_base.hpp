#ifndef STRUCTURE_BASE_HPP
#define STRUCTURE_BASE_HPP

#include <iostream>
#include <vector>

class Structure {
protected:
    int vertexAmount;
    int edgeAmount;

public:
    Structure(/* args */);
    ~Structure();
    virtual bool hasEdgeUV(int V, int U) = 0;
    virtual std::vector<int> getAdjList(int V) = 0;
    virtual void resize(int size) = 0;
    virtual void insertEdge(int u,int V,double w) = 0;
    virtual double getWeightUV(int V, int U) = 0;
    virtual void printGraph() = 0;
    
};

Structure::Structure(/* args */): vertexAmount(0), edgeAmount(0)
{
}

Structure::~Structure()
{
}


#endif