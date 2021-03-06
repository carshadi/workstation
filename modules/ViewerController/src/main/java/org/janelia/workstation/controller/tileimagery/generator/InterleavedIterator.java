package org.janelia.workstation.controller.tileimagery.generator;

import java.util.Iterator;

/**
 * Alternately generates values from two iterators, until both are exhausted.
 * @author brunsc
 * @param <E>
 *
 */
public class InterleavedIterator<E> implements Iterator<E>, Iterable<E> {
	private Iterator<E> first;
	private Iterator<E> second;
	private boolean useFirst = true;
	
	public InterleavedIterator(Iterable<E> first, Iterable<E> second) {
	    if (first != null)
	        this.first = first.iterator();
	    if (second != null)
	        this.second = second.iterator();
	}

	@Override
	public boolean hasNext() {
		return (first.hasNext() || second.hasNext());
	}

	@Override
	public E next() {
		Iterator<E> primary, secondary;
		if (useFirst) {
			primary = first;
			secondary = second;
		} else {
			primary = second;
			secondary = first;
		}
		useFirst = ! useFirst; // swap for next time
		if (primary.hasNext()) {
			return primary.next();
		} else {
			// primary is exhausted
			return secondary.next();
		}
	}

	@Override
	public void remove() {
		throw new UnsupportedOperationException(); // because I'm lazy
	}

	@Override
	public Iterator<E> iterator() {
		return this;
	}
}
