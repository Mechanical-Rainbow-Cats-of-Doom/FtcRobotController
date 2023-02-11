package org.firstinspires.ftc.teamcode.core.softwaretools;

import java.io.Serializable;
import java.util.LinkedList;

public class CircularlyLinkedList<T> implements Serializable {
    public static class Node<T> implements Serializable {
        private T val;
        private Node<T> nextNode;
        public Node(T val) {
            this.val = val;
        }

        public Node<T> getNextNode() {
            return nextNode;
        }
        
        public void setNextNode(Node<T> nextNode) {
            this.nextNode = nextNode;
        }
        
        public T getVal() {
            return val;
        }

        public void setVal(T val) {
            this.val = val;
        }
    }
    private final Node<T> head;
    private Node<T> curNode, tail;
    private final int capacity;
    private int size = 1;

    public CircularlyLinkedList(int capacity, T initVal) {
        this.capacity = capacity;
        this.head = new Node<>(initVal);
        head.setNextNode(head);
        this.curNode = head;
        this.tail = head;
    }

    public CircularlyLinkedList(T initVal) {
        this(Integer.MAX_VALUE, initVal);
    }

    public Node<T> getHead() {
        return head;
    }

    public Node<T> getCurNode() {
        return curNode;
    }

    public Node<T> getTail() {
        return tail;
    }

    public Node<T> get(int index) {
        int iter = 0;
        Node<T> node = getHead();
        while (++iter < index) {
            node = node.getNextNode();
        }
        return node;
    }

    public void set(int index, T val) {
        get(index).setVal(val);
    }
    public void add(T val) {
        if (size++ < capacity) {
            Node<T> newNode = new Node<>(val);
            tail.setNextNode(newNode);
            tail = newNode;
            tail.setNextNode(head);
        } else {
            curNode.setVal(val);
            curNode = curNode.getNextNode();
        }
    }

    public int length() {
        return size;
    }

}
