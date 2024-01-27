package Path;

// Edge
public class Segment {
	private static int total = 0;
	private int id;
//	Maybe instead of vertexes, simple integers which are identified only by ids.
	private int start; // u
	private int end; // v
	private double length; // Must be non-negative
	private double speedLimit; // Must be non-negative
	
	public Segment(int start, int end, double length, double speedLimit) {
		this.start = start;
		this.end = end;
		this.length = length;
		this.speedLimit = speedLimit;
		id = total;
		total++;
	}
	
	public boolean isBefore(Segment seg) {
		return end == seg.start;
	}

	public boolean crosses(Segment seg) {
		return start == seg.start;
	}

	public void printSegment() {
		System.out.println("The segment " + this.toString() + ", with length " + length + " and speed limit " + speedLimit);
	}

	@Override
	public String toString() {
		return "(" + start + ", " + end + ")";
	}

	public double getSpeedLimit() {
		return speedLimit;
	}

	public double getLength() {
		return length;
	}

	public int getId() {
		return id;
	}
	
}
