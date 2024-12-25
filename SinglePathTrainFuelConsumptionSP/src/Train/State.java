package Train;

import java.util.ArrayList;

import Path.Segment;

public class State {
	private Position pos;
	private double speed;
	
	public State(Position position, double speed) {
		pos = position;
		this.speed = speed;
	}
//	In paper but not used at least till Algorithm 2
	public boolean isValidFor(Train t) {
		if(!(speed >=0 && speed <= t.getMaxSpeed())) {
			return false;
		}
		
		return pos.isValidFor(t, speed);
	}

	public double getSpeed() {
		return speed;
	}

	public int getHSgm() {
		return pos.getHSgm();
	}
	
	public int getTSgm() {
		return pos.getTSgm();
	}

	public double getDist() {
		return pos.getDist();
	}

	public double getTotalLength() {
		return pos.getTotalLength();
	}

//	In paper but not used at least till Algorithm 2
	public double getMinSL() {
		return pos.getMinSL();
	}

	public ArrayList<Segment> getSegments(int tSgm, int hSgm) {
		return pos.getSegments(tSgm, hSgm);
	}
	
	public void print() {
		pos.print();
		System.out.println("Speed: " + speed);
	}
}
