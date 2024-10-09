#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <fstream>
#include <string>
#include <queue>



class Graph
{
private:
    int edge;
    bool directed;
    int size;
public:
    Graph(bool directed = false);
    ~Graph();
    void setSize(int n);
    void importEDG(const std::string& path);
};





#endif