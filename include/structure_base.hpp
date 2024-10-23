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
#include <unordered_map>

using IntVec = std::vector<int>;
using PairVec = std::vector<std::pair<int, std::optional<double>>>;
using ReturnType = std::variant<IntVec, PairVec>;


class AdjMatrix;
class AdjVector;
class Graph;


class Structure {

    private:

        /**
         * @brief Helper initiator for weightless graphs
         * @param line A string of format "%d %d" that represents an edge of the graph
         * @param isDirected A boolean that indicates if `Graph` is directed or not.
         */
        void helper_init(std::string line, bool isDirected);



        /**
         * @brief Helper initiator for weighted graphs.
         * @param line A string of format "%d %d %f" that represents an edge of the graph and its weight.
         * 
         * If this edge contains a weight of value = 0.0, it should indicate this and not just omit the weight
         * in the string, as this will throw an error
         * 
         * @param isDirected A boolean that indicates if `Graph` is directed or not.
         * 
         * @todo Missing implementation.
         */
        void helper_initWeighted(std::string line, bool isDirected);

        

    protected:
    
        /**
         * @brief A struct that stores information about a vertex.
         * 
         * Currently, it holds the vertex degree but can be extended to include more attributes as needed.
         */
        struct Data {
            int degree = 0;
            std::string name;

            Data() = default;

            /// Allows access to Structure class and its childs
            friend class Structure;
            friend class AdjMatrix;
            friend class AdjVector;
            
        };

        /// Class attibutes ///
        int vertexAmount;
        int edgeAmount;
        bool isWeighted;
        std::unordered_map<int, Data> data; ///< Map storing vertex data by vertex ID.

        


        /**
         * @brief Initializes the `Graph` structure.
         * 
         * @param path A string of the path to graph source. 
         * @param isDirected A boolean that indicates if `Graph` is directed or not.
         * @param isWeighted A boolean that indicates if `Graph` edges have weight or not.
         * 
         * @throw std::invalid_argument If isDirected = true as this library does
         *        not support directed graphs yet.
         */
        void initStruct(const std::string &path, bool isDirected, bool isWeighted);


        /**
         * @brief  Insert edge (U,V) in `Graph` with weight w
         * 
         * Must be implemented in each graph representation class.
         * 
         * @param U Source vertex ID.
         * @param V Destination vertex ID.
         * @param w Weight of the edge (U,V).
         */
        virtual void insertEdge(int U,int V,double w = 1.0) = 0;

        /**
         * @brief Initializes the Data struct that stores the graph.
         * 
         * This method is responsible for preparing the internal structure
         * that stores graph information, ensuring it can handle the given number of vertices.
         * Must be implemented in each graph representation class.
         * 
         * @param size The number of vertices in the graph.
         */
        virtual void resize(int size) = 0;

        /**
         * @brief Constructs a new instance of the Structure class.
         * 
         * This constructor is protected to prevent direct instantiation of the Structure class.
         * Derived classes should call this constructor.
         */
        Structure() = default;

    public:

        /**
         * @brief Virtual destructor for the Structure class.
         * 
         * Ensures that the derived class destructors are called properly when 
         * an object is deleted through a pointer to the base class.
         */
        virtual ~Structure() = default;

        /**
         * @brief Gets a map of vertices data
         * 
         * Each vertice is mapped by their ID (e.g., data[U]).
         * The Data struct contain all information for a given vertex. 
         * 
         * @return An unordered_map of all vertices data.
         */
         std::unordered_map<int, Data> getDataArray();

        

        /**
         * @brief Checks if graph G has an edge from vertex U to vertex V.
         * 
         * Must be implemented in each graph representation class.
         * 
         * @param V Source vertex ID.
         * @param U Destination vertex ID.
         * @return Returns true if G has the edge (U,V), false otherwise.
         */
        virtual bool hasEdgeUV(int V, int U) = 0;

        /**
         * @brief Gets the degree of a specified vertex.
         * 
         * @param U The vertex ID.
         * @return The degree value of vertex U.
         */
        int getUDegree(int U);
        
        /**
         * @brief Tells if `Graph` edges have weight.
         * @return 
         * - true if `Graph` edges have weight.
         * - false otherwise.
         */
        bool hasWeight();

        /**
         * @brief Gets the weight of the edge (U,V).
         * 
         * @param U Source vertex ID.
         * @param V Destination vertex ID.
         * @return Double weight of edge (U,V).
         * 
         * @throw std::logic_error If this method is called on a weightless graph.
         * @throw std::invalid_argument if the Graph does not contain edge (U, V).
         */
        virtual double getWeightUV(int V, int U) = 0;
        

        /**
         * @brief Gets a list of all edges in `Graph`.
         * 
         * 
         * 
         * @return An data structure containing all edges of `Graph`.
         * 
         * @todo find out if this method is necessary, if so still needs implementation.
         */
        //virtual std::unordered_map<std::optional<double>, std::pair<int, int>> getEdgeList();
        
        /**
         * @brief Gets a list of all adjacent vertices of U.
         * 
         * @param U Source vertex ID.
         * @return 
         * - A vector containing the IDs of all vertices adjacent to vertex U.
         * - An empty vector if U has no adjacent vertices (degree[U] = 0).
         */
        virtual std::vector<int> getAdjArray(int U) = 0;

        /**
         * @brief Gets a list of all adjacent vertices of U along with their weights. 
         * 
         * @param U Source vertex ID.
         * @return 
         * - A vector containing the IDs and weight of all vertices adjacent to vertex U.
         * - An empty vector if U has no adjacent vertices (degree[U] = 0).
         * 
         * @throw std::logic_error If this method is called on a weightless `Graph`.
         */
        virtual std::vector<std::pair<int, double>> getAdjWeightedArray(int U) = 0;

        /**
         * @brief Gets the amount of vertices in `Graph`
         * @return The amount of vertices in `Graph`
         */
        int getVertexAmount();

        /**
         * @brief Gets the amount of edges in `Graph`
         * @return The amount of edges in `Graph`
         */
        int getEdgeAmount();

        
    
};


#endif