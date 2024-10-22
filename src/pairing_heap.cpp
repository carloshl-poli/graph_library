#include "pairing_heap.hpp"

// Helper to merge two heaps
template <typename T>
typename PairingHeap<T>::Node* PairingHeap<T>::meld(Node* h1, Node* h2) {
    if (!h1) return h2;
    if (!h2) return h1;
    if (h1->key < h2->key) {
        h2->prev = h1;
        h2->sibling = h1->child;
        if (h1->child) h1->child->prev = h2;
        h1->child = h2;
        return h1;
    } else {
        h1->prev = h2;
        h1->sibling = h2->child;
        if (h2->child) h2->child->prev = h1;
        h2->child = h1;
        return h2;
    }
}

// Helper to meld all siblings when deleting the minimum
template <typename T>
typename PairingHeap<T>::Node* PairingHeap<T>::meldSiblings(Node* node) {
    if (!node || !node->sibling) return node;

    std::vector<Node*> treeArray;

    // Pair nodes two by two
    while (node) {
        Node* a = node;
        Node* b = node->sibling;
        node = node->sibling ? node->sibling->sibling : nullptr;
        a->sibling = nullptr;
        if (b) b->sibling = nullptr;
        treeArray.push_back(meld(a, b));
    }

    // Meld all resulting trees
    for (size_t i = 1; i < treeArray.size(); ++i) {
        treeArray[0] = meld(treeArray[0], treeArray[i]);
    }
    return treeArray[0];
}

// Destructor to clean up
template <typename T>
PairingHeap<T>::~PairingHeap() {
    clear(root);
}

// Insert a new key
template <typename T>
typename PairingHeap<T>::Node* PairingHeap<T>::insert(T key) {
    Node* newNode = new Node(key);
    root = meld(root, newNode);
    return newNode;
}

// Find the minimum key
template <typename T>
T PairingHeap<T>::findMin() const {
    if (!root) throw std::runtime_error("Heap is empty.");
    return root->key;
}

// Delete the minimum key
template <typename T>
void PairingHeap<T>::deleteMin() {
    if (!root) throw std::runtime_error("Heap is empty.");

    Node* oldRoot = root;
    if (!root->child) {
        root = nullptr;
    } else {
        root = meldSiblings(root->child);
        root->prev = nullptr;
    }
    delete oldRoot;
}

// Decrease the key of a given node
template <typename T>
void PairingHeap<T>::decreaseKey(Node* node, T newKey) {
    if (newKey >= node->key) throw std::invalid_argument("New key is not smaller.");

    node->key = newKey;
    if (node == root) return;

    // Remove node from its current position
    if (node->prev) {
        if (node->prev->child == node) {
            node->prev->child = node->sibling;
        } else {
            node->prev->sibling = node->sibling;
        }
    }
    if (node->sibling) {
        node->sibling->prev = node->prev;
    }

    // Meld the node back to the root
    node->prev = nullptr;
    node->sibling = nullptr;
    root = meld(root, node);
}

// Check if the heap is empty
template <typename T>
bool PairingHeap<T>::isEmpty() const {
    return root == nullptr;
}

// Clear the heap
template <typename T>
void PairingHeap<T>::clear(Node* node) {
    if (!node) return;
    clear(node->child);
    clear(node->sibling);
    delete node;
}
