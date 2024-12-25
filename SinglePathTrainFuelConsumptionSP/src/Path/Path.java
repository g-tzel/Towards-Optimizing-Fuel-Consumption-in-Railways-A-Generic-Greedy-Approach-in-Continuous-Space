package Path;
import java.util.ArrayList;

import Train.Train;

public class Path {
	private static int total = 0;
	private int id;
	ArrayList<Segment> segments;
	
	public Path() {
		segments = new ArrayList<Segment>();
		id = total;
		total++;
	}
	
	public void addSegment(Segment seg) {
		if(segments.isEmpty()) {
			segments.add(seg);
			return;
		}
		
		if(isConsecutive(seg) && !createsCross(seg)) {
			segments.add(seg);
			return;
		}
		
		System.out.println("The segment " + seg.toString() + " cannot be added to the path.");
	}

//	Checks if seg creates cross with the already added segments
//	Doesn't allow u1==uk. No cycle allowed
	private boolean createsCross(Segment seg) {
		for(Segment s : segments){
			if(s.crosses(seg)) {
				return true;
			}
		}
		return false;
	}

//	Checks if seg (new Segment) is right after the last added segment
	private boolean isConsecutive(Segment seg) {
		return segments.get(segments.size()-1).isBefore(seg);
	}

//	In paper but not used at least till Algorithm 2
	public boolean isValidFor(Train t, double speed) {
		for(Segment seg : segments) {
			if(seg.getSpeedLimit() < speed) {
				return false;
			}
		}
		return true;
	}

	public int getHSgm() {
		return segments.get(segments.size()-1).getId(); //segments.size()-1;
	}

	public int getTSgm() {
		return segments.get(0).getId();
	}
	
	public double getLengthOf(int i) {
		return segments.get(i).getLength();
	}

	public double getTotalLength() {
		double total = 0;
		for(Segment seg : segments) {
			total += seg.getLength();
		}
		return total;
	}
//	In paper but not used at least till Algorithm 2
	public double getMinSL() {
		double min = segments.get(0).getSpeedLimit();
		for(int i = 1; i < segments.size(); i++) {
			double speedLimit = segments.get(i).getSpeedLimit();
			if(min > speedLimit) {
				min = speedLimit;
			}
		}
		return min;
	}

	public ArrayList<Segment> getSegments(int tSgm, int hSgm) {
		ArrayList<Segment> segs = new ArrayList<Segment>();
		for(int i = tSgm; i <= hSgm; i++) {
			segs.add(segments.get(i));
		}
		return segs;
	}
	
	public void print() {
		System.out.println("The path " + this.toString() + " consists of the segments: ");
		System.out.println("--------------------------------");
		for(Segment seg : segments) {
			seg.printSegment();
		}
	}
	
	@Override
	public String toString() {
		return "(" + id + ")";
	}
}
