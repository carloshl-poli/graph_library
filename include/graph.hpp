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
#include <cmath>

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

    struct Data{
        int level;
        int parent;

        Data(int level, int parent) : parent(parent), level(level) {}

        friend class Graph;
    };

public:
    using ReturnGraphDataMap = std::unordered_map<int, Graph::Data>;

    Graph(): vertexAmount(0), directed(false), hasWeight(false){}
    Graph(std::string &path, GraphStructure structure, bool isDirected = false);
    ~Graph() = default;

    //Graph Basic Data Methods
    double getMean();
    double getMedian();
    int getMinDegree();
    int getMaxDegree();

    //Graph Export Methods
    void graphExportAsEDG(std::string &path);
    void graphExportAsData(std::string &path);
    void subCompExportAsStream(std::string &path);

    //Graph Basic Algorithm
    ReturnGraphDataMap getBFSTree(int U);
    ReturnGraphDataMap  getDFSTree(int U);
    ReturnGraphDataMap  getDijkstraTree(int U, bool useHeap = true);
    ReturnGraphDataMap  getPrimmMST();
    ReturnGraphDataMap  getKruskalMST();

    ReturnGraphDataMap  getBellmanFordTree(int U);
    ReturnGraphDataMap  getFloydWarshallTree(int U);



    //Graph Basic Measure Methods
    std::optional<int> getUVDistance(int U, int V);
    int getApproxDiameter();
    int getExactDiameter();


    //Graph Utility Methods
    std::priority_queue<int, std::unordered_map<int, std::pair<int, int>>> getGraphSubComp();

};





#endif