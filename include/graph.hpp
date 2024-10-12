#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <string>
#include "structure_base.hpp"
#include "adj_matrix.hpp"
#include "adj_vector.hpp"
#include <memory>
#include <deque>
#include <stack>
#include <vector>
#include <unordered_map>
#include <variant>
#include <optional>
#include <queue>

using VertexDataVec = std::variant<std::unordered_map<int, std::pair<int, int>>,
                      std::deque<int>>;


enum class GraphStructure{
    AdjMatrix,
    AdjVector
};

struct VertexTempData {
    int vertex;
    int parent;
    int level;
};

class Graph {
protected:
    int vertexAmount;
    bool directed;
    bool hasWeight;
    std::unique_ptr<Structure> structure;
    int edgeAmount;

public:
    Graph(): vertexAmount(0), directed(false), hasWeight(false){}
    Graph(std::string &path, GraphStructure structure, bool isDirected = false);
    ~Graph() = default;

    //Graph Basic Data Methods
    double getMean(bool weighted = false);
    double getMedian(bool weighted = false);
    double getMinDegree(bool weighted = false);
    double getMaxDegree(bool weighted = false);

    //Graph Export Methods
    void graphExportAsEDG(std::string &path);
    void graphExportAsData(std::string &path);
    void subCompExportAsStream(std::string &path);

    //Graph Basic Algorithm
    std::unordered_map<int, std::pair<int, int>>  getUBFSTree(int U);
    std::unordered_map<int, std::pair<int, int>>  getUDFSTree(int U);


    //Graph Basic Measure Methods
    std::optional<double> getUVDistance(int U, int V);
    double getGraphDiameter(bool getAccurate = true);


    //Graph Utility Methods
    std::priority_queue<int, std::unordered_map<int, std::pair<int, int>>> getGraphSubComp();

};





#endif