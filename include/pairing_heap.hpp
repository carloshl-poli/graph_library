#ifndef PAIRING_HEAP_HPP
#define PAIRING_HEAP_HPP

#include "utilities.hpp"
#include <unordered_map>
#include <limits>
#include <iostream>
#include <queue>
#include <stack>
#include <memory>

struct HeapNode {
    float key;  // Valor associado (distância, por exemplo)
    int vertex; // Vértice associado
    std::shared_ptr<HeapNode> child, foward, back;

    HeapNode() {
        this->key = 0;
        this->vertex = 0;
    }
    HeapNode(float key, int vertex) {
        this->key = key;
        this->vertex = vertex;
    }
};

class PairingHeap {
private:
    std::shared_ptr<HeapNode> root;  // Raiz do heap
    std::unordered_map<int, std::shared_ptr<HeapNode>> heapMap;  // Mapeia os vértices para os nós do heap

    std::shared_ptr<HeapNode> first();

public:
    PairingHeap() = default;
    PairingHeap(std::shared_ptr<HeapNode> node) : root(std::move(node)) {}

    // Inserir um novo vértice com um valor
    void insert(int vertex, float key);

    void merge(std::shared_ptr<PairingHeap> ph);

    

    // Retorna o vértice de menor valor (find-min)
    int findMin() const;

    // Remove o vértice de menor valor (delete-min)
    std::shared_ptr<HeapNode> extractMin();

    void deleteMin();


    // Diminui a chave de um vértice no heap
    void decreaseKey(int vertex, float newKey);

    // Verifica se o heap está vazio
    bool isEmpty() const;


    // Limpa o heap e libera memória
    void clear();
};

#endif