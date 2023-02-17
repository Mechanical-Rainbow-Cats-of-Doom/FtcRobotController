package org.firstinspires.ftc.teamcode.core.softwaretools;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

public class CircularlyLinkedList<T> implements Serializable, Collection<T> {
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
    public boolean remove(@Nullable Object o) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean containsAll(@NonNull Collection<?> c) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean addAll(@NonNull Collection<? extends T> c) {
        c.forEach(this::add);
        return true;
    }

    @Override
    public boolean removeAll(@NonNull Collection<?> c) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean retainAll(@NonNull Collection<?> c) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void clear() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int size() {
        return size;
    }

    @Override
    public boolean isEmpty() {
        return size != 0;
    }

    @Override
    public boolean contains(@Nullable Object o) {
        return this.stream().anyMatch(a -> a == o);
    }

    public static class ListIterator<T> implements Iterator<T> {
        Node<T> current, head;
        private boolean iterated = false;

        public ListIterator(@NonNull CircularlyLinkedList<T> list) {
            head = list.getHead();
            current = head;
        }

        public ListIterator(Node<T> node) {
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

    @NonNull
    @Override
    public Iterator<T> iterator() {
        return new ListIterator<>(this);
    }

    @NonNull
    @Override
    public Object[] toArray() {
        return new ArrayList<>(this).toArray();
    }

    @NonNull
    @Override
    public <T1> T1[] toArray(@NonNull T1[] a) {
        throw new UnsupportedOperationException(); // this is bad
    }

}
