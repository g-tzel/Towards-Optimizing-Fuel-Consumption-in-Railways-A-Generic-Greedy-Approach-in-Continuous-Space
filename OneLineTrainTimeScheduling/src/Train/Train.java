package Train;

public class Train {
	private double length;
	private double maxSpeed;
	private double maxAcc;
	private double maxDec;
	
	public Train(double length, double maxSpeed, double maxAcc, double maxDec) {
		this.length = length;
		this.maxSpeed = maxSpeed;
		this.maxAcc = maxAcc;
		this.maxDec = maxDec;
	}
	
//	How do you implement them? Maybe in Position?
	public void head() {}
	public void tail() {}

	public void print() {
		System.out.println("Characteristics of train " + this.toString() + " :");
		System.out.println("Length: " + length + " meters");
		System.out.println("Max Speed: " + maxSpeed + "m/s");
		System.out.println("maxAcc: " + maxAcc + "m/s\u00b2");
		System.out.println("maxDec: " + maxDec + "m/s\u00b2");
	}

	public double getMaxSpeed() {
		return maxSpeed;
	}

	public double getLength() {
		return length;
	}

	public double getMaxAcc() {
		return maxAcc;
	}

	public double getMaxDec() {
		return maxDec;
	}
}