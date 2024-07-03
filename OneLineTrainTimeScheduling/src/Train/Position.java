package Train;

import java.util.ArrayList;

import Path.Path;
import Path.Segment;

public class Position {
	private Path span; // The path the train spans over
	private double dist; // x: The distance of head(T) from vk across sk
	
	public Position(ArrayList<Segment> segments, double dist) {
		span = new Path();
		for(Segment seg : segments) {			
			span.addSegment(seg);
		}
		this.dist = dist;
	}

	public Position(Segment seg, double dist) {
		span = new Path();
		span.addSegment(seg);
		this.dist = dist;
	}
//	In paper but not used at least till Algorithm 2
	public boolean isValidFor(Train t, double speed) {
		return span.isValidFor(t, speed);
	}

	public int getHSgm() {
		return span.getHSgm();
	}

	public int getTSgm() {
		return span.getTSgm();
	}
	
	public double getDist() {
		return dist;
	}

	public double getTotalLength() {
		return span.getTotalLength();
	}
//	In paper but not used at least till Algorithm 2
	public double getMinSL() {
		return span.getMinSL();
	}

	public ArrayList<Segment> getSegments(int tSgm, int hSgm) {
		return span.getSegments(tSgm, hSgm);
	}

	public void print() {
		span.print();
		System.out.println("Dist: " + dist);
	}
}
