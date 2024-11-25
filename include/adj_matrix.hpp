#ifndef ADJ_MATRIX_HPP
#define ADJ_MATRIX_HPP

#include "structure_base.hpp"
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <utility>
#include <stdexcept>
#include <optional>

class AdjMatrix : public Structure {
    private:
        /**
         * @brief The body of the structure representation.
         * 
         * Basically consists of a Matrix of vectors optional<double>
         * The value inside the matrix represents the weight of the edge that incide in the vertices of
         * the matrix indices.
         * To represent the absence of edge, the value stored is a nullopt
         */
        std::vector<std::vector<std::optional<double>>> body;

    protected:
        /**
         * @brief Sets the matrix to be (size x size)
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
         * this method just call init_struct with the param it receives.
         * 
         * @param path 
         * @param isWeighted 
         * @param isDirected 
         */
        AdjMatrix(const std::string &path, bool isWeighted, bool isDirected = false);

        /**
         * @brief Default destructor
         */
        ~AdjMatrix () = default;

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
         * @brief Gets a data structure that contains all vertices that are adjacent to U
         * 
         * Complexity O(n)
         * 
         * @param U 
         * @return An array containing all vertices adjacent to U
         * 
         * Change this method to return a iterator in order to reduce
         * complexity to O(1). 
         */
        std::vector<int> getAdjArray(int U) override;

        /**
         * @brief Gets a data structure that contains all vertices (as well as the weight of
         * the edge that inscides them) that are adjacent to U.
         * 
         * Each element of this data strcuture is a pair of an integer and a double.
         * The integer represents the vertice value.
         * The double represents the weight of the egde that inscides it and U
         * 
         * Complexity O(n)
         * 
         * @param U 
         * @return 
         */
        std::vector<std::pair<int, double>> getAdjWeightedArray(int U) override;

        std::string getType();

};





#endif