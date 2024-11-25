#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <string>
#include "structure_base.hpp"
#include "adj_matrix.hpp"
#include "adj_vector.hpp"
#include "pairing_heap.hpp"
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

/**
 * @brief Enum that represents the structures available in the library.
 */
enum class GraphStructure{
    AdjMatrix,
    AdjVector
};

/**
 * @brief Struct used in general methods that constructs a map of all vertices data.
 * @deprecated Replaced to more specifics structs for each method
 */
struct VertexTempData {
    int vertex;
    int parent;
    int level;
};

class Graph {
    private:

        /**
         * @brief Structure use do store the reference to a subgraph.
         * Used to construct a data structure of all subgraphs in a way that
         * they will be ordered by their size.
         */
        struct MaxHeapComparator {
            bool operator()(const std::pair<int, std::vector<int>>& a,
                            const std::pair<int, std::vector<int>>& b) {
                return a.first < b.first;
            }
        };

        /**
         * @brief A map of all label's vertices.
         * 
         * Used to find the vertice related to a label.
         */
        std::unordered_map<std::string, int> labelToVertexMap;

        /**
         * @brief A map of all vertice's labels.
         * 
         * Used to find the label related to a vertice.
         */
        std::unordered_map<int, std::string> vertexToLabelMap;


        /**
         * @brief A helper method to be used only by the method that finds the
         * diameter of the graph
         * 
         * @param U 
         * @param mark A map of all current markations of all vertices in the graph.
         * @param markReference The value that represents that the vertice is marked.
         *  
         * @return 
         */
        int helper_Diameter(int U, std::unordered_map<int, int>& mark, int markReference = 1);

        /**
         * @brief Template used to quickly make a map of all vertices with any type
         * @tparam T Type of the Value of the map
         * @param value Value of the map
         * @return A map of <int, T>
         */
        template <typename T>
        std::unordered_map<int, T> initVertexMap(T value)
        {
            std::unordered_map<int, T> markMap;
            for (int V = 1; V <= vertexAmount; V++){
                markMap[V] = value;
            }
            return markMap;
        }
        

        
        /**
         * @brief A routine that is shared between all different constructors
         * 
         * @param path A string of the path to the graph file
         * @param structure Which structure should be used in the graph
         * @param isDirected A boolean that indicates if the graph is directed or not.
         * @param isWeighted A boolean that indicates if the graph is weighted or not.
         * 
         * @deprecated Replaced by setting a base constructor that will do this.
         */
        void helper_init(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted);

        


    protected:

        /**
         * @brief Struct used in general methods that constructs a map of all vertices data.
         * @deprecated Replaced to more specifics structs for each method
         */
        struct Data{
            int level;
            int parent;

            Data(int level, int parent) : parent(parent), level(level) {}
            Data() : parent(-1), level(-1) {}

            friend class Graph;
        };

        /**
         * @brief Stores the size of the graph
         */
        int vertexAmount;

        /**
         * @brief Indicates if the graph is directed or not.
         */
        bool directed;

        /**
         * @brief Indicates if the graph is weighted or not.
         */
        bool hasWeight;

        /**
         * @brief Indicates if the graph is labeled or not.
         * 
         * @note The name "name" is not been used anymore in favour "labed" in 
         * more recent methods.
         */
        bool named;

        /**
         * @brief Stores the number of edges in the graph
         */
        int edgeAmount;

        //auto getDataArray(Structure& obj);
    public:

        /**
         * @brief A Structure that is used on the dijkstra methods.
         * 
         * It rerepresents the data that can be found during a dijkstra algorithm 
         * routine.
         * 
         * A map of this struct can be used to find the min distance and path of
         * all vertices in a weighted graph.
         */
        struct DijkstraNode {
            int parent;
            double distance;

            DijkstraNode() : parent(-1), distance(-1) {}

            friend class Graph;
        };

        /**
         * @brief Stores the address of the structure of the graph
         * 
         * @note In order to increase safety of the library, should be considerate
         * in the future if this member shouldn't be only access by exterior
         * throgh a get method.
         */
        std::unique_ptr<Structure> structure;
        
        /**
         * @brief A group of abreviation of big type Names in order to increase
         * readability of the methods. Some of them All deprecated.
         */
        using ReturnGraphDataMap = std::unordered_map<int, Graph::Data>;
        using nodeData = std::variant<DijkstraNode, Data>;
        using GraphDataMap = std::unordered_map<int, nodeData>;
        using ReturnSubGraphHeap = std::priority_queue<std::pair<int, std::vector<int>>,
                                   std::vector<std::pair<int, std::vector<int>>>,
                                   Graph::MaxHeapComparator>;
        
        /**
         * @brief A default Constructor for when you instance the graph without passing any param.
         * 
         * @note This constructor has no practical directive use for the user, but could become
         * if the library is extended to be able to add vertices and edges to existing graphs.
         */
        Graph(): vertexAmount(0), directed(false), hasWeight(false){}

        /**
         * @brief Basic constructor of the graph
         * 
         * initialize all of graph members.
         * The graph that it uses must be of the format of:
         * - First line must be a single integer representing the size of the graph
         * - All subsequent lines represents an edege of the graph
         * 
         * @param path A string of the path to the graph file
         * @param structure Which structure should be used in the graph
         * @param isDirected A boolean that indicates if the graph is directed or not.
         * @param isWeighted A boolean that indicates if the graph is weighted or not.
         * 
         * @note Should be called by other constructors.
         * 
         * @todo Extends the library to support default EDG type graph files.
         */
        Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted);

        /**
         * @brief overloading of the constructor to support labeled graphs.
         * 
         * This constructor first calls the basic constructor and then initialize
         * the members of the graph related to the label.
         * 
         * The label should be on the format "i.label" in which i should the
         * index of the label.
         * 
         * @param path A string of the path to the graph file
         * @param structure Which structure should be used in the graph
         * @param isDirected A boolean that indicates if the graph is directed or not.
         * @param isWeighted A boolean that indicates if the graph is weighted or not. 
         * @param names A string of the path to the label file 
         */
        Graph(std::string &path, GraphStructure structure, bool isDirected, bool isWeighted, std::string &names);

        /**
         * @brief Default Constructor of the class.
         */
        ~Graph() = default;


        //Graph Basic Data Methods

        /**
         * @brief Gets the mean of all degrees in the graph.
         * @return A double representing the mean of degrees.
         */
        double getMean();
        /**
         * @brief Gets the median of all degrees in the graph.
         * @return A double representing the median of degrees.
         */
        double getMedian();

        /**
         * @brief Gets the minimal degree in the graph.
         * @return An integer representing the minimal degree of graph.
         */
        int getMinDegree();

        /**
         * @brief Gets the maximum degree in the graph.
         * @return An integer representing the maximum degree of graph.
         */
        int getMaxDegree();


        //Graph Export Methods

        /**
         * @brief Export this graph to a format of EDG file.
         * 
         * This format does not countain the size of the graph in the file.
         * 
         * @param path The path in which the EDG will be stored.
         * 
         * @note This method is not yet implemented.
         */
        void graphExportAsEDG(std::string &path);

        /**
         * @brief Export this graph to a format of txt file.
         * 
         * This format countain the size of the graph in the first line of the
         * file.
         * 
         * @param path The path in which the txt file will be stored.
         * 
         * @note This method is not yet implemented.
         */
        void graphExportAsData(std::string &path);

        /**
         * @brief Export. to a log file, all useful data of every subgraph in this graph.
         * 
         * The file is structured in decrescent order of size of the subgraphs.
         * 
         * @param subGraphHeap A dataStructure containing all subgraphs in decrescent order of size.
         * @param filename The name of the file that will be stored this data.
         * If such file doens't exist, one will be created with this name.
         */
        void printSubGraphsToFile(const ReturnSubGraphHeap& subGraphHeap, const std::string& filename);

        /**
         * @brief Export all data retrieved from a BFS/DFS routine to a log file.
         * 
         * The file is a table of all vertices that were reached by the search in this form:
         * - Vertex value
         * - Parent of the vertex
         * - Level of the vertex
         * 
         * @param searchTree A data structure returned by a BFS/DFS routine
         * @param filename The name of the file that will be stored this data.
         * If such file doens't exist, one will be created with this name. 
         * @param root The root used in the search routine.
         */
        void printSearchTreeToFile(const ReturnGraphDataMap &searchTree, const std::string &filename, int root);

        /**
         * @brief Export valued data of the graph
         * 
         * @param graph The address of the graph (usually &this).
         * @param toFile Indicates if the data should be printed to a log (true) or to the console (false).
         * @param filename If the data shpiçd be printed on a log, this name should be passed.
         */
        void printGraphStats(const Graph &graph, bool toFile, const std::string &filename);
        
        


        //Graph Basic Algorithm

        /**
         * @brief Gets a Data Structure builded during a search from a Vertex
         * 
         * The elements in this structure is composed of:
         * - parent An integer representing the vertex parent in the tree
         * - level An integer representing the vertex level in the tree
         * 
         * @param U The source of the search
         * @return A Data Structure containing all reached vertex data.
         * 
         */
        ReturnGraphDataMap getBFSTree(int U);
        /**
         * @brief Same as the above, but for a DFS. 
         */
        ReturnGraphDataMap  getDFSTree(int U);

        /**
         * @brief Gets a Data Structure builded during a Dijkstra algorithm execution.
         * 
         * The elements in this structure is composed of:
         * - parent An integer representing the vertex parent in the tree
         * - distance An boolean representing the vertex distance in the tree
         * 
         * @param U The source of the dijkstra algorithm.
         * @param useHeap Indicates if should be used a heap in the algorithm.
         * @return A Data Structure containing all reached vertex data.
         * 
         * @throw std::logic_error if this graph isn't weighted.
         * @throw std::logic_error if the search reach a negative edge.
         */
        std::unordered_map<int, DijkstraNode> getDijkstraTree(int U, bool useHeap);
        /**
         * @brief Same as above, but without using heap.
         */
        std::unordered_map<int, DijkstraNode> getDijkstraTreeDefault(int U);

        /**
         * @brief gets the minimum spanning tree builded from a Primm algoritm
         * 
         * @return A data structure capable of building a MST of this graph.
         * 
         * @note This method is not yet implemmented.
         */
        ReturnGraphDataMap  getPrimmMST();

        /**
         * @brief gets the minimum spanning tree builded from a Kruskal algoritm
         * 
         * @return A data structure capable of building a MST of this graph.
         * 
         * @note This method is not yet implemmented.
         */

        ReturnGraphDataMap  getKruskalMST();
        /**
         * @note This method is not yet implemmented and not yet known what it
         * should return.
         */
        ReturnGraphDataMap  getBellmanFordTree(int U);

        /**
         * @note This method is not yet implemmented and not yet known what it
         * should return.
         */
        ReturnGraphDataMap  getFloydWarshallTree(int U);



        //Graph Basic Measure Methods
        
        /**
         * @brief Gets the number of edges between vertices U and V.
         * 
         * If there is no valid path between them, them their distance is infinite.
         * 
         * @param U 
         * @param V 
         * @return The distance of U and V.
         */
        int getUVDistance(int U, int V);

        /**
         * @brief Gets the weighted distance of vertices U and V.
         * 
         * It calls the dijkstra method in vertex U and tries to reach distance of V. 
         * If V is not in the dijkstra tree, then there is no valid path between them
         * and infinite is returned.
         * 
         * @param U 
         * @param V 
         * @param useHeap Indicates if should be used a heap in the dijkstraTree routine
         * @return The distance between U and V.
         * 
         * @deprecated Thi method was replaced by getDijkstraMinDistance, which has a
         * better flexibility and usuability.
         */
        double getUVDistance(int U, int V, bool useHeap);

        /**
         * @brief Gets the weighted distance of vertices U and V.
         * 
         * It uses the dijkstra method, without making a copy, to find the distance between
         * vertices U and V.
         * 
         * If V is not in the dijkstra tree, then there is no valid path between them
         * and infinite is returned.
         * 
         * Different from the method it replaces, this one does not build and destroy the
         * dijkstra tree during its execution, but use an already made one so the tree can
         * be re-used in other dijkstra methos.
         *  
         * @param dijkstraTree The addres of the dijkstra tree
         * @param V 
         * @return The distance between U and V.
         */
        double getDijkstraMinDistance(std::unordered_map<int, DijkstraNode> &dijkstraTree, int V);

        /**
         * @brief Gets the weighted distance of vertices U and V.
         * 
         * 
         * An overloading of getDijkstraMinDistance method, focusing on cases which
         * the tree will not be re-used.
         * Is an adaptation of the replaced method getUVDistance.
         * Now it builds a dijkstra tree and calls the basic method with the param given
         * 
         * @param U 
         * @param V 
         * @param useHeap Indicates if should be used a heap in the dijkstraTree routine
         * @return The distance between U and V.
         */
        double getDijkstraMinDistance(int U, int V, bool useHeap);


        
        
        
        int getApproxDiameter();
        int getExactDiameter();

        //Graph Utility Methods
        ReturnSubGraphHeap getGraphSubComp();
        int getVertexAmount();
        int getEdgeAmount();
        std::stack<int> getPathUV(int U, int V, bool useHeap);
        
        std::queue<std::string> getNamedPathUV(std::string name1, std::string name2);

        int getVertexByLabel(std::string name);
        std::string getLabelByVertex(int vertex);

        std::deque<int> getDijkstraMinPathVertices(int U, int V, bool useHeap);
        std::deque<int> getDijkstraMinPathVertices(int U, int V, std::unordered_map<int, DijkstraNode> &dijkstraTree);

        std::deque<std::string> getDijkstraMinPathLabels(std::string labelU, std::string labelV, bool useHeap);
        std::deque<std::string> getDijkstraMinPathLabels(std::string labelU, std::string labelV, std::unordered_map<int, DijkstraNode> &dijkstraTree);

        //std::unordered_map<int, DijkstraNode> getDijkstraTree(int U, bool useHeap, bool StopBeforeConditionMet, int V);
        

        //Templates



        

};





#endif


