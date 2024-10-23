#ifndef PAIRING_HEAP_HPP
#define PAIRING_HEAP_HPP

#include <unordered_map>
#include <limits>
#include <iostream>

struct HeapNode {
    int value;  // Valor associado (distância, por exemplo)
    int vertex; // Vértice associado
    HeapNode* child;
    HeapNode* sibling;

    HeapNode(int val, int v) : value(val), vertex(v), child(nullptr), sibling(nullptr) {}
};

class PairingHeap {
private:
    HeapNode* root;  // Raiz do heap
    std::unordered_map<int, HeapNode*> heapMap;  // Mapeia os vértices para os nós do heap

    // Função auxiliar para mesclar dois heaps
    HeapNode* meld(HeapNode* a, HeapNode* b);

    // Mescla todos os filhos de um nó
    HeapNode* mergeSiblings(HeapNode* node);

public:
    PairingHeap();

    // Inserir um novo vértice com um valor
    void insert(int vertex, int value);

    // Retorna o vértice de menor valor (find-min)
    int findMin() const;

    // Remove o vértice de menor valor (delete-min)
    void deleteMin();

    // Diminui a chave de um vértice no heap
    void decreaseKey(int vertex, int newValue);

    // Verifica se o heap está vazio
    bool isEmpty() const;

    // Limpa o heap e libera memória
    void clear();
};

#endif
