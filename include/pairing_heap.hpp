#ifndef PAIRING_HEAP_H
#define PAIRING_HEAP_H

#include <iostream>
#include <vector>
#include <limits>

template <typename T>
class PairingHeap {
private:
    struct Node {
        T key;
        Node* child;
        Node* sibling;
        Node* prev;

        Node(T k) : key(k), child(nullptr), sibling(nullptr), prev(nullptr) {}
    };

    Node* root;

    // Meld two heaps
    Node* meld(Node* h1, Node* h2);

    // Meld all siblings (used for deleteMin)
    Node* meldSiblings(Node* node);

public:
    PairingHeap() : root(nullptr) {}
    ~PairingHeap();

    // Insert a new key
    Node* insert(T key);

    // Find the minimum key
    T findMin() const;

    // Delete the minimum key
    void deleteMin();

    // Decrease the key of a given node
    void decreaseKey(Node* node, T newKey);

    // Check if the heap is empty
    bool isEmpty() const;

    // Clear the heap
    void clear(Node* node);
};

#endif 
