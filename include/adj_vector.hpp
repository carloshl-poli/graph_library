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
        

        int getEdgeUV(int U, int V);
    protected:
        void resize(int size) override;

        void insertEdge(int U, int V, double w) override;
        size_t insertEdge(int U, int V, double W, bool getID);

    public:
        AdjVector(size_t size, bool isWeighted);
        AdjVector(const std::string &path, bool isWeighted, bool isDirected = false);
        ~AdjVector () = default;
        friend class FlowGraph;


        bool hasEdgeUV(int U, int V) override;
        double getWeightUV(int U, int V) override;

        std::vector<int> getAdjArray(int U) override;
        std::vector<std::pair<int, double>> getAdjWeightedArray(int U) override;
        std::string getType();

        std::vector<NodeBase> getAdjNodeArray(int U);

    
};


#endif