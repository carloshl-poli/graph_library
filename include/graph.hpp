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
#include <algorithm>
#include <numeric>

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

struct Data{
    int level;
    int parent;

    Data(int level, int parent) : parent(parent), level(level) {}

    friend class Graph;
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
    std::optional<double> getMinDegree(bool weighted = false);
    std::optional<double> getMaxDegree(bool weighted = false);

    //Graph Export Methods
    void graphExportAsEDG(std::string &path);
    void graphExportAsData(std::string &path);
    void subCompExportAsStream(std::string &path);

    //Graph Basic Algorithm
    std::unordered_map<int, Data>  getBFSTree(int U);
    std::unordered_map<int, std::pair<int, int>>  getDFSTree(int U);
    std::unordered_map<int, std::pair<int, int>>  getDijkstraTree(int U, bool useHeap = true);
    std::unordered_map<int, std::pair<int, int>>  getPrimmMST();
    std::unordered_map<int, std::pair<int, int>>  getKruskalMST();

    std::unordered_map<int, std::pair<int, int>>  getBellmanFordTree(int U);
    std::unordered_map<int, std::pair<int, int>>  getFloydWarshallTree(int U);



    //Graph Basic Measure Methods
    int getUVDistance(int U, int V);
    int getGraphDiameter(bool getAccurate = true);


    //Graph Utility Methods
    std::priority_queue<int, std::unordered_map<int, std::pair<int, int>>> getGraphSubComp();

};





#endif