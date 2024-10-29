#include "pairing_heap.hpp"

// Inserir um vértice no heap
void PairingHeap::insert(int vertex, float key) {
    auto newNode = std::make_shared<HeapNode>(key, vertex);
    auto newHeap = std::make_shared<PairingHeap>(newNode);
    this->merge(newHeap);
    heapMap[vertex] = newNode;
}

void PairingHeap::merge(std::shared_ptr<PairingHeap> ph) {
    if (ph->isEmpty()) return;

    if (this->isEmpty()) {
        this->root = ph->root;
        return;
    }
    else {
        if (this->root->key < ph->root->key){
            auto firstChild = this->root->child;
            this->root->child = ph->root;
            ph->root->back = this->root;
            ph->root->foward = firstChild;
            if (firstChild != nullptr) firstChild->back = ph->root;
            
        }
        else {
            auto firstChild = ph->root->child;
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

std::shared_ptr<HeapNode> PairingHeap::first(){
    return this->root;
}

// Remover o vértice de menor valor
std::shared_ptr<HeapNode> PairingHeap::extractMin() {
    auto deletedNode = this->first();
    auto deletedChildsList = deletedNode->child;

    if (deletedChildsList == nullptr) {
        this->root = nullptr;
        return deletedNode;
    }

    std::queue<std::shared_ptr<PairingHeap>> heapQueue;
    std::stack<std::shared_ptr<PairingHeap>> heapStack;

    while (deletedChildsList != nullptr) {
        auto next = deletedChildsList->foward;
        auto ChildsListHeap = std::make_shared<PairingHeap>(deletedChildsList);
        heapQueue.push(ChildsListHeap);
        deletedChildsList = next;
    }

    while (!heapQueue.empty()){
        auto temp1 = heapQueue.front();
        heapQueue.pop();
        if (!heapQueue.empty()) {
            auto temp2 = heapQueue.front();
            heapQueue.pop();
            temp1->merge(temp2);
        }
        heapStack.push(temp1);
    }

    while (heapStack.size() > 1) {
        auto temp3 = heapStack.top();
        heapStack.pop();
        auto temp4 = heapStack.top();
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

    auto minNode = this->extractMin();
}

// Diminuir a chave de um vértice
void PairingHeap::decreaseKey(int vertex, float newKey) {
    auto node = heapMap[vertex];
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
    auto newHeap = std::make_shared<PairingHeap>(node);
    this->merge(newHeap);
}



// Verifica se o heap está vazio
bool PairingHeap::isEmpty() const {
    return this->root == nullptr;
}

// Limpar o heap e liberar memória
void PairingHeap::clear() {
    this->root = nullptr;
    this->heapMap.clear();
}