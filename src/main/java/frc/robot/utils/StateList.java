package frc.robot.utils;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.function.Predicate;

/**
 * Returns TRUE if the predicate value returns true for all elements
 *  in the length limited LinkedList.
 * 
 * StateList<Boolean> state = StateList.bool();
 * state.add(true);
 * state.get();
 */
public class StateList<T> {

	private final LinkedList<T> list;
	private final int maxLength;
	private final Predicate<T> predicate;

	public static StateList<Boolean> bool() {
		return StateList.bool(10);
	}

	public static StateList<Boolean> bool(int maxLength) {
		StateList<Boolean> newInstance = new StateList<Boolean>(val -> val == true, maxLength);
		return newInstance;
	}

	public StateList(Predicate<T> predicate) {
		this(predicate, 10);
	}

	public StateList(Predicate<T> predicate, int maxLength) {
		this.predicate = predicate;
		this.maxLength = maxLength;

		this.list = new LinkedList<T>();
	}

	public StateList<T> add(T entry) {
		if (this.list.size() >= this.maxLength) {
			this.list.removeFirst();
		}
		this.list.add(entry);
		return this;
	}

	public boolean get() {
		if (this.list.size() != this.maxLength) return false;

		return this.list.stream().filter(this.predicate).count() == this.list.size();
	}

	public ArrayList<T> values() {
		return new ArrayList<T>(this.list);
	}

}
