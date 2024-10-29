#ifndef PAIRING_HEAP_HPP
#define PAIRING_HEAP_HPP

#include <unordered_map>
#include <limits>
#include <iostream>
#include <queue>
#include <stack>
struct HeapNode {
    double key;  // Valor associado (distância, por exemplo)
    int vertex; // Vértice associado
    HeapNode *child, *foward, *back;

    HeapNode() {
        this->key = 0;
        this->vertex = 0;
        this->child = this->foward = this->back = nullptr;
    }
    HeapNode(double key, int vertex) {
        this->key = key;
        this->vertex = vertex;
        this->child = this->foward = this->back = nullptr;
    }
};

class PairingHeap {
private:
    HeapNode* root;  // Raiz do heap
    std::unordered_map<int, HeapNode*> heapMap;  // Mapeia os vértices para os nós do heap

    HeapNode* first();

public:
    PairingHeap();
    PairingHeap(HeapNode *node);

    // Inserir um novo vértice com um valor
    void insert(int vertex, double key);

    void merge(PairingHeap *ph);

    

    // Retorna o vértice de menor valor (find-min)
    int findMin() const;

    // Remove o vértice de menor valor (delete-min)
    HeapNode* extractMin();

    void deleteMin();


    // Diminui a chave de um vértice no heap
    void decreaseKey(int vertex, double newKey);

    // Verifica se o heap está vazio
    bool isEmpty() const;

    void deleteAllNodes(HeapNode* node);

    // Limpa o heap e libera memória
    void clear();
};

#endif
