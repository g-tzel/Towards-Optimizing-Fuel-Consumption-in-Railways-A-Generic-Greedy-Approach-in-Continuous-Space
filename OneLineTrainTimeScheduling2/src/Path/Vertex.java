package Path;

// Node
public class Vertex {
//	Coordinates (x,y)
	private int xcord;
	private int ycord;
	
	public Vertex(int xcord, int ycord) {
		this.xcord = xcord;
		this.ycord = ycord;
		}
	
	@Override
	public boolean equals(Object obj) {
		Vertex v = (Vertex) obj;
		return xcord == v.xcord && ycord == v.ycord;
	}
	
	@Override
	public String toString() {
		return "(" + xcord + ", " + ycord + ")";
	}
	
	public int getXcord() {
		return xcord;
	}
	public int getYcord() {
		return ycord;
	}
}
