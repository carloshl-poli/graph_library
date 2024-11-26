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
#include <unordered_map>

class AdjVector : public Structure {
    public:
        struct NodeBase {
            int origin;
            double weight;
            int to;
            size_t id;

            NodeBase(int origin, double weight, int to, size_t id) : origin(origin), weight(weight), to(to), id(id) {}
        };

        size_t id_counter;
        
    private:
        std::vector<std::vector<NodeBase>> body;
        //std::vector<std::vector<std::pair<int, double>>> body;
        

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

        void insertEdge(int U, int V, double w) override;
        size_t insertEdge(int U, int V, double W, bool getID);

    public:
        AdjVector(size_t size, bool isWeighted);
        AdjVector(const std::string &path, bool isWeighted, bool isDirected = false);

        /**
         * @brief Default destructor
         */
        ~AdjVector () = default;
        friend class FlowGraph;


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
        std::string getType();

        std::vector<NodeBase> getAdjNodeArray(int U);

    
};


#endif