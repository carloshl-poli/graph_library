#include "pairing_heap.hpp"

// Construtor do PairingHeap
PairingHeap::PairingHeap() : root(nullptr) {}

// Função para mesclar dois heaps
HeapNode* PairingHeap::meld(HeapNode* a, HeapNode* b) {
    if (!a) return b;
    if (!b) return a;
    if (a->value < b->value) {
        b->sibling = a->child;
        a->child = b;
        return a;
    } else {
        a->sibling = b->child;
        b->child = a;
        return b;
    }
}

// Inserir um vértice no heap
void PairingHeap::insert(int vertex, int value) {
    HeapNode* newNode = new HeapNode(value, vertex);
    root = meld(root, newNode);
    heapMap[vertex] = newNode;
}

// Encontrar o vértice de menor valor
int PairingHeap::findMin() const {
    if (root) {
        return root->vertex;
    }
    return -1;  // Retorna -1 se o heap estiver vazio
}

// Remover o vértice de menor valor
void PairingHeap::deleteMin() {
    if (!root) return;

    HeapNode* oldRoot = root;
    root = mergeSiblings(root->child);  // Mescla os filhos da raiz removida
    heapMap.erase(oldRoot->vertex);     // Remove o vértice da raiz do mapa
    delete oldRoot;  // Libera memória
}

// Mesclar todos os irmãos de um nó (operação de delete-min)
HeapNode* PairingHeap::mergeSiblings(HeapNode* node) {
    if (!node || !node->sibling) return node;

    // Mescla dois a dois
    HeapNode* first = node;
    HeapNode* second = node->sibling;
    HeapNode* rest = second->sibling;

    first->sibling = nullptr;
    second->sibling = nullptr;

    return meld(meld(first, second), mergeSiblings(rest));
}

// Diminuir a chave de um vértice
void PairingHeap::decreaseKey(int vertex, int newValue) {
    HeapNode* node = heapMap[vertex];
    if (!node || newValue >= node->value) return;

    // Remove o nó atual e substitui pelo novo valor
    node->value = newValue;

    // Se não é a raiz, remover e mesclar de volta
    if (node != root) {
        if (node->sibling) node->sibling = nullptr;  // Desconectar dos irmãos
        root = meld(root, node);
    }
}

// Verifica se o heap está vazio
bool PairingHeap::isEmpty() const {
    return root == nullptr;
}

// Limpar o heap e liberar memória
void PairingHeap::clear() {
    while (!isEmpty()) {
        deleteMin();
    }
}
