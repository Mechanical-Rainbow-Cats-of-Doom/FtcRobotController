package org.firstinspires.ftc.teamcode.core.softwaretools;

import androidx.annotation.NonNull;

import java.util.AbstractCollection;
import java.util.Iterator;

public class CircularlyLinkedList<T> extends AbstractCollection<T> {
    public static class Node<T> {
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

    public Node<T> getNode(int index) {
        int iter = 0;
        Node<T> node = getHead();
        while (++iter < index) {
            node = node.getNextNode();
        }
        return node;
    }

    public T get(int index) {
        return getNode(index).getVal();
    }

    public void set(int index, T val) {
        getNode(index).setVal(val);
    }

    @Override
    public boolean add(T val) {
        if (size++ < capacity) {
            Node<T> newNode = new Node<>(val);
            tail.setNextNode(newNode);
            tail = newNode;
            tail.setNextNode(head);
        } else {
            curNode.setVal(val);
            curNode = curNode.getNextNode();
        }
        return true;
    }

    @Override
    public int size() {
        return size;
    }

    @Override
    public boolean isEmpty() {
        return size != 0;
    }

    public static class OnceIterator<T> implements Iterator<T> {
        Node<T> current, head;
        private boolean iterated = false;

        public OnceIterator(@NonNull CircularlyLinkedList<T> list) {
            head = list.getHead();
            current = head;
        }

        public OnceIterator(Node<T> node) {
            current = node;
        }

        @Override
        public boolean hasNext() {
            return !iterated || current != head;
        }

        @Override
        public T next() {
            iterated = true;
            T ret = current.getVal();
            current = current.getNextNode();
            return ret;
        }
    }

    public static class CircularIterator<T> extends OnceIterator<T> {
        public CircularIterator(@NonNull CircularlyLinkedList<T> list) {
            super(list);
        }

        public CircularIterator(Node<T> node) {
            super(node);
        }

        @Override
        public boolean hasNext() {
            return true;
        }
    }

    @NonNull
    @Override
    public Iterator<T> iterator() {
        return new OnceIterator<>(this);
    }
}
