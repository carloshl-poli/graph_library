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

#define OUTPUT_PATH "../output/"
using VertexDataVec = std::variant<std::unordered_map<int, std::pair<int, int>>,
                      std::deque<int>>;

class Structure;

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
    private:
        int Graph::helper_Diameter(int U, std::unordered_map<int, int>& mark, int markReference = 1);
        template <typename T>
        std::unordered_map<int, T> initVertexMap(T value)
        {
            std::unordered_map<int, T> markMap;
            for (int V = 1; V <= vertexAmount; V++){
                markMap[V] = value;
            }
            return markMap;
        }
        struct MaxHeapComparator {
            bool operator()(const std::pair<int, std::vector<int>>& a,
                            const std::pair<int, std::vector<int>>& b) {
                return a.first < b.first;
            }
        };

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

        auto getDataArray(Structure& obj);
    public:
        using ReturnGraphDataMap = std::unordered_map<int, Graph::Data>;
        using ReturnSubGraphHeap = std::priority_queue<std::pair<int, std::vector<int>>, std::vector<std::pair<int, std::vector<int>>>, Graph::MaxHeapComparator>;

        Graph(): vertexAmount(0), directed(false), hasWeight(false){}
        Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted);
        ~Graph() = default;

        //Graph Basic Data Methods
        double getMean();
        double getMedian();
        int getMinDegree();
        int getMaxDegree();

        //Graph Export Methods
        void graphExportAsEDG(std::string &path);
        void graphExportAsData(std::string &path);
        void printSubGraphsToFile(const ReturnSubGraphHeap& subGraphHeap, const std::string& filename);
        void printSearchTreeToFile(const ReturnGraphDataMap &searchTree, const std::string &filename, int root);
        void printGraphStats(const Graph &graph, bool toFile, const std::string &filename);

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
        ReturnSubGraphHeap getGraphSubComp();

};





#endif


