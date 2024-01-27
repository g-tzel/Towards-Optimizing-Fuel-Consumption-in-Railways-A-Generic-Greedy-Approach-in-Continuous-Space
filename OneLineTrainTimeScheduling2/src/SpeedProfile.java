import java.util.ArrayList;

import Path.Segment;
import Train.State;
import Train.Train;

public class SpeedProfile {
	private ArrayList<State> states;
	private ArrayList<T> timelabels;
	
	public SpeedProfile(State state, double time) {
		states = new ArrayList<State>();
		timelabels = new ArrayList<T>();
		addState(state, time);
	}
	
	public SpeedProfile() {
		states = new ArrayList<State>();
		timelabels = new ArrayList<T>();
	}
	
//	In paper but not used at least till Algorithm 2
	public boolean isValidFor(Train t) {
		for(State st : states) {
			if(!st.isValidFor(t)) {
				return false;
			}
		}
		
		return true;
	}
	
//	In paper but not used at least till Algorithm 2
	public State stateAt(double t) {
		for(int i = 0; i < states.size(); i++) {
			if(t >= timelabels.get(i).mint() && t <= timelabels.get(i).maxt()) {
				return getState(i);
			}
		}
		return null;
	}
	
//	In paper but not used at least till Algorithm 2
	public double mint(double t) {
		for(int i = 0; i < states.size(); i++) {
			if(t >= timelabels.get(i).mint() && t <= timelabels.get(i).maxt()) {
				return timelabels.get(i).mint();
			}
		}
		
		return -1;
	}

//	In paper but not used at least till Algorithm 2
	public double maxt(double t) {
		for(int i = 0; i < states.size(); i++) {
			if(t >= timelabels.get(i).mint() && t <= timelabels.get(i).maxt()) {
				return timelabels.get(i).maxt();
			}
		}
		
		return -1;
	}
	
	class T{
		private double t1;
		private double t2;
		
		public T(double t1, double t2) {
			this.t1 = t1;
			this.t2 = t2;
		}
		
		public double mint() {
			return t1;
		}
		
		public double maxt() {
			return t2;
		}

		public void print() {
			System.out.println("Starts at: " + t1);
			System.out.println("Ends at: " + t2);
		}
	}

	public void addState(State state, double time) {
		states.add(state);
		double t1;
		if(timelabels.size() == 0) {
			t1 = 0;
		}else {
			t1 = timelabels.get(timelabels.size()-1).maxt();
		}
		double t2 = time;
		timelabels.add(new T(t1, t2));
	}

	public State getLastState() {
		return states.get(states.size()-1);
	}

	public ArrayList<Segment> getSegments(int tSgm, int hSgm) {
		return getLastState().getSegments(tSgm, hSgm);
	}

	public int nOfEntries() {
		return states.size();
	}

	public double getSpeedOf(int i) {
		return getState(i).getSpeed();
	}

	public SpeedProfile keepStates(int threshold) {
		SpeedProfile newSp = new SpeedProfile();
		for(int i = 0; i <= threshold; i++) {
			newSp.addState(states.get(i), timelabels.get(i).maxt());
		}
		return newSp;
	}

	public State getState(int i) {
		return states.get(i);
	}
	
	public void print() {
		System.out.println("\n\n\n\n______________________________");
		System.out.println("The solution: ");
		for(int i = 0; i < states.size(); i++) {
			states.get(i).print();
			timelabels.get(i).print();
		}
	}

	public double getTimeOfState(int i) {
		return timelabels.get(i).maxt();
	}
	
}
