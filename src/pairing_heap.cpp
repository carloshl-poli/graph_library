#include "pairing_heap.hpp"

// Construtor do PairingHeap
PairingHeap::PairingHeap() : root(nullptr) {}

PairingHeap::PairingHeap(HeapNode *node) {
    this->root = node;
    node->foward = node->back = nullptr;
}

// Inserir um vértice no heap
void PairingHeap::insert(int vertex, double key) {
    HeapNode* newNode = new HeapNode(key, vertex);
    this->merge(new PairingHeap(newNode));
    heapMap[vertex] = newNode;
}

void PairingHeap::merge(PairingHeap *ph) {
    if (ph->isEmpty()) return;

    if (this->isEmpty()) {
        this->root = ph->root;
        return;
    }
    else {
        if (this->root->key < ph->root->key){
            HeapNode *firstChild = this->root->child;
            this->root->child = ph->root;
            ph->root->back = this->root;
            ph->root->foward = firstChild;
            if (firstChild != nullptr) firstChild->back = ph->root;
            
        }
        else {
            HeapNode *firstChild = ph->root->child;
            ph->root->child = this->root;
            this->root->back = ph->root;
            this->root->foward = firstChild;
            if (firstChild != nullptr) firstChild->back = this->root;
            this->root = ph->root;
        }
    }
}

// Encontrar o vértice de menor valor
int PairingHeap::findMin() const {
    if (root != nullptr) {
        return root->vertex;
    }
    return -1;  // Retorna -1 se o heap estiver vazio
}

HeapNode* PairingHeap::first(){
    return this->root;
}

// Remover o vértice de menor valor
HeapNode* PairingHeap::extractMin() {
    HeapNode *deletedNode = this->first();
    HeapNode *deletedChildsList = deletedNode->child;

    if (deletedChildsList == nullptr) {
        this->root = nullptr;
        return deletedNode;
    }

    std::queue<PairingHeap*> heapQueue;
    std::stack<PairingHeap*> heapStack;

    while (deletedChildsList != nullptr) {
        HeapNode *next = deletedChildsList->foward;
        heapQueue.push(new PairingHeap(deletedChildsList));
        deletedChildsList = next;
    }

    while (!heapQueue.empty()){
        PairingHeap *temp1 = heapQueue.front();
        heapQueue.pop();
        if (!heapQueue.empty()) {
            PairingHeap *temp2 = heapQueue.front();
            heapQueue.pop();
            temp1->merge(temp2);
        }
        heapStack.push(temp1);
    }

    while (heapStack.size() > 1) {
        PairingHeap *temp3 = heapStack.top();
        heapStack.pop();
        PairingHeap *temp4 = heapStack.top();
        heapStack.pop();
        temp3->merge(temp4);
        heapStack.push(temp3);

    }

    this->root = heapStack.top()->root;
    heapMap.erase(deletedNode->vertex);

    return deletedNode;
}

void PairingHeap::deleteMin() {
    if (this->isEmpty()) {
        std::logic_error("Error! the Heap is already empty.");
    }

    HeapNode* minNode = this->extractMin();
    delete minNode;
}

// Diminuir a chave de um vértice
void PairingHeap::decreaseKey(int vertex, double newKey) {
    HeapNode* node = heapMap[vertex];
    node->key = newKey;

    //Case node is root of Heap
    if (this->root == node) return;


    //Case node is firstChild of a node
    if (node->back->child == node) {
        node->back->child = node->foward;
        if (node->foward != nullptr) node->foward->back = node->back;
    }
    else {
        node->back->foward = node->foward;
        if (node->foward != nullptr) node->foward->back = node->back;
    }

    this->merge(new PairingHeap(node));
}



// Verifica se o heap está vazio
bool PairingHeap::isEmpty() const {
    return this->root == nullptr;
}

void PairingHeap::deleteAllNodes(HeapNode *node) {
    if (node == nullptr) return;

    HeapNode* child = node->child;
    while (child != nullptr) {
        HeapNode* nextChild = child->foward;
        deleteAllNodes(child);
        child = nextChild;
    }

    delete node;
}

// Limpar o heap e liberar memória
void PairingHeap::clear() {
    this->deleteAllNodes(this->root);
    this->root = nullptr;
    this->heapMap.clear();
}
