package org.firstinspires.ftc.teamcode.core.softwaretools;

import androidx.annotation.Nullable;

public class CircularlyLinkedList<T> {
    public static class Node<T> {
        private T val;
        private Node<T> nextNode;
        public Node(T val) {
            this.val = val;
        }

        public Node<T> getNextNode() {
            return nextNode;
        }

        public T getVal() {
            return val;
        }

        public void setVal(T val) {
            this.val = val;
        }
    }
    private Node<T> head, curNode;
    private Node<T> tail = null;
    private final int capacity;
    private int size = 1;

    public CircularlyLinkedList(int capacity, T initVal) {
        this.capacity = capacity;
        this.head = new Node<>(initVal);
        this.curNode = head;
    }

    public CircularlyLinkedList(T initVal) {
        this(Integer.MAX_VALUE, initVal);
    }

    public Node<T> getHead() {
        return head == null ? null : head;
    }

    public Node<T> getCurNode() {
        return curNode;
    }

    @Nullable
    public Node<T> getTail() {
        return tail == null ? null : tail;
    }

    public void add(T val) {
        if (size++ < capacity) {
            Node<T> newNode = new Node<>(val);
            if (head == null) {
                head = newNode;
            } else {
                tail.nextNode = newNode;
            }
            tail = newNode;
            tail.nextNode = head;
        } else {
            curNode.setVal(val);
            curNode = curNode.nextNode;
        }
    }

    public int length() {
        return size;
    }

}
