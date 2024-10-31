#ifndef ADJ_VECTOR_HPP
#define ADJ_VECTOR_HPP

#include "structure_base.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <optional>

class AdjVector : public Structure
{
    private:

        /**
         * @brief The body of the structure representation.
         * 
         * Basically consists of a Vector of pointers to a vector of adjacency.
         * Each index of the vector represents a vertex of the graph.
         * Since the graph is 1-indexation, the index of the graph in the vector
         * of pointers is the vertice value minus 1.
         * 
         * Each element of the vector of adjacency is a std::pair<int, double>,
         * in which the first is the value of the vertex (not the index) and
         * the second represents the weight of the edge between them.
         * 
         */
        std::vector<std::vector<std::pair<int, double>>> body;

        /**
         * @brief Gets the location of the vertice V in the adjacency vector of U.
         * 
         * This method basically iterates in the adjacency vector of U until it
         * finds the location of V.
         * It has private access since it should only be used by other class methods.
         * 
         * @param U 
         * @param V 
         * @return The index of V in U adjacent vector.
         * 
         * @throw std::invalid_argument if the edge (U,V) don't exist.
         */
        int getEdgeUV(int U, int V);


    protected:
        /**
         * @brief Sets the size of the vector of pointers to size
         * 
         * @param size 
         */
        void resize(int size) override;

        /**
         * @brief Insert the edge (u,v) in the body with weight w
         * 
         * @param u 
         * @param V 
         * @param w 
         */
        void insertEdge(int u,int V,double w) override;

    public:

        /**
         * @brief The constructor of the class
         * 
         * Since the construction of the structure lies with the method initStruct of its base class,
         * this method just call initStruct with the param it receives.
         * 
         * @param path 
         * @param isWeighted 
         * @param isDirected 
         */
        AdjVector(const std::string &path, bool isWeighted, bool isDirected = false);

        /**
         * @brief Default destructor
         */
        ~AdjVector () = default;

        /**
         * @brief Verifies if exist an edge that incides U and V.
         * 
         * @param U 
         * @param V 
         * @return Returns a boolean of the verification
         */
        bool hasEdgeUV(int U, int V) override;

        /**
         * @brief Gets the weight of the edge that incides U and V.
         * 
         * @param U 
         * @param V 
         * @return The weight of the edge
         * 
         * @throw std::logic_error if this is called in a weightless graph.
         * @throw std::invalid_argument if the edge (U,V) don't exist.
         */
        double getWeightUV(int U, int V) override;

        /**
         * @brief Gets a data structure that contains all vertices that are adjacent to U.
         * 
         * This method iterates the adjacency vector of U and construct a vector
         * of all vertices in this vector.
         * 
         * Since it visits each edge that incides U, it has a complexity of O(m).
         * 
         * @param U 
         * @return An array containing all vertices adjacent to U.
         * 
         * @todo Change this method to return a iterator in order to reduce
         * complexity to O(1). 
         */
        std::vector<int> getAdjArray(int U) override;

        /**
         * @brief Gets a data structure that contains all vertices (as well as the weight of
         * the edge that inscides them) that are adjacent to U.
         * 
         * Each element of this data strcuture is a pair of an integer and a double.
         * The integer represents the vertice value.
         * The double represents the weight of the egde that inscides it and U.
         * 
         * Since this pair is basically the element of the adjacent Vector, this
         * method basically returns a copy of such vector, making the complexity
         * equal to Complexity O(1).
         * 
         * @param U 
         * @return 
         */
        std::vector<std::pair<int, double>> getAdjWeightedArray(int U) override;
};


#endif