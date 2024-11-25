#ifndef FLOW_GRAPH_HPP
#define FLOW_GRAPH_HPP

#include "graph.hpp"
#include "adj_vector.hpp"
#include <unordered_map>
#include <string>
#include <stack>
#include <functional>
#include <bit>
#include <cmath>

class FlowGraph : public Graph {
    private:
        std::unique_ptr<AdjVector> structure;
    public:
        
        struct FlowEdge {
            std::string edge;
            double capacity;
            double flow;

            FlowEdge() : edge(""), capacity(0.0), flow(0.0) {}
            FlowEdge(std::string edge, double capacity, double flow) : edge(edge), capacity(capacity), flow(flow) {}
        };
        struct ResidualEdge {
            std::string originalEdge;
            size_t id;
            int origin;
            int destiny;
            double capacity;
            size_t sisterID;
            ResidualEdge() : originalEdge(""), id(0), origin(0), destiny(0), capacity(0.0), sisterID(0) {}
            ResidualEdge(std::string originalEdge, size_t id, int origin, int destiny, double capacity) : 
                originalEdge(originalEdge), id(id), origin(origin), destiny(destiny), capacity(capacity) {}
        };

        FlowGraph(std::string &path, bool isDirected, bool isWeighted);
        AdjVector initResidualStructure(std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap);
        std::unordered_map<size_t, FlowGraph::ResidualEdge> initResidualStructure();
        std::stack<FlowGraph::ResidualEdge> findAugmentingPath(int source,
            int target, int minCapacity, std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap,
             AdjVector residualStructure, int &bottleneck);

        void helper_FordFulkerson(std::unordered_map<size_t, ResidualEdge> &residualEdgeMap,
            AdjVector &residualStructure, std::unordered_map<std::string, FlowGraph::FlowEdge> &flowEdgeMap);

        void Augment(std::stack<ResidualEdge> path, std::unordered_map<std::string, FlowEdge> &flowEdgeMap,
            int bottleneck, std::unordered_map<size_t, ResidualEdge> &ResidualEdgeMap);
        std::unordered_map<std::string, FlowEdge> FordFulkerson(int source, int target);
        int calculateMaxFlow(const std::unordered_map<std::string, FlowEdge>& flowEdgeMap, const int source);
        int calculateMaxFlow(const int source, const int target);

        
};

#endif