#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <string>
#include "structure_base.hpp"
#include "adj_matrix.hpp"
#include "adj_vector.hpp"
#include "pairing_heap.hpp"
#include "utilities.hpp"
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
#include <limits>
#include <map>
#include <unordered_set>

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
        

        int helper_Diameter(int U, std::unordered_map<int, int>& mark, int markReference = 1);
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

        std::unordered_map<int, std::string> nameMap;

        void helper_init(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted);

        


    protected:
        int vertexAmount;
        bool directed;
        bool hasWeight;
        
        int edgeAmount;

        struct Data{
            int level;
            int parent;

            Data(int level, int parent) : parent(parent), level(level) {}
            Data() : parent(-1), level(-1) {}

            friend class Graph;
        };

        //auto getDataArray(Structure& obj);
    public:

        std::unique_ptr<Structure> structure; //TEMP
        struct DijkstraNode {
            int parent;
            float distance;

            DijkstraNode() : parent(-1), distance(-1) {}

            friend class Graph;
        };

        using ReturnGraphDataMap = std::unordered_map<int, Graph::Data>;
        using nodeData = std::variant<DijkstraNode, Data>;
        using GraphDataMap = std::unordered_map<int, nodeData>;
        using ReturnSubGraphHeap = std::priority_queue<std::pair<int, std::vector<int>>,
                                   std::vector<std::pair<int, std::vector<int>>>,
                                   Graph::MaxHeapComparator>;
        
        Graph(): vertexAmount(0), directed(false), hasWeight(false){}
        Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted);
        Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted, std::string &names);
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
        std::unordered_map<int, DijkstraNode> getDijkstraTree(int U, bool useHeap);
        std::unordered_map<int, DijkstraNode> getDijkstraTree(int U);
        ReturnGraphDataMap  getPrimmMST();
        ReturnGraphDataMap  getKruskalMST();

        ReturnGraphDataMap  getBellmanFordTree(int U);
        ReturnGraphDataMap  getFloydWarshallTree(int U);



        //Graph Basic Measure Methods
        int getUVDistance(int U, int V);
        float getUVDistance(int U, int V, bool useHeap);
        
        int getApproxDiameter();
        int getExactDiameter();

        //Graph Utility Methods
        ReturnSubGraphHeap getGraphSubComp();
        int getVertexAmount();
        int getEdgeAmount();
        std::stack<int> getPathUV(int U, int V, bool useHeap);

        //Templates

        template <typename T>
        std::vector<T> getNamedPath(int U, int V, bool useHeap){
            std::vector<T> namedPath;
            auto path = this->getPathUV(U, V, useHeap);
            while (!path.empty()) {
                namedPath.emplace_back(nameMap[path.top()]);
                path.pop();
            }
            return namedPath;
        }

        

};





#endif


