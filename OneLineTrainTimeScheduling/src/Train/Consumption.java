package Train;

import Main.SpeedProfile;

public class Consumption {
	private static final double kv=1; // Speed Weight (could be static or not)
	private static final double ka=1; // Acceleration Weight
	private static final double kb=1; // Deceleration Weight
	private static double deltaT;
	
	public static double calcConsumtion(SpeedProfile sp) {
		double total = 0;
		for(int i = 1; i < sp.nOfEntries(); i++) {
			State state1 = sp.getState(i-1);
			State state2 = sp.getState(i);
			double startSpeed = state1.getSpeed();
			double endSpeed = state2.getSpeed();
			double t1 = sp.getStartTimeAtState(i);
			double t2 = sp.getEndTimeAtState(i);
			deltaT = t2-t1;
			total += spotSpeedCons(startSpeed, endSpeed, deltaT);
			System.out.println(total);
		}
		return total;
	}
	
	private static double spotSpeedCons(double startSpeed, double endSpeed, double deltaT) {
		double Wv = 0;
		double a = (endSpeed - startSpeed)/(deltaT);
		
		Wv =  kv * ((2*startSpeed + a*(deltaT))/2)*(deltaT);
		double result = Wv + calcWl(a, deltaT);
		if(result<0) {
			return 0;
		}
		return result;
	}
	
	private static double calcWl(double a, double deltaT) {
		double Wl = acc(a, deltaT);
		if(a < 0) {
			Wl = dec(a, deltaT);
		}
		return Wl;
	}

	private static double acc(double a, double deltaT) {
		return ka * a * (deltaT);
	}
	
	private static double dec(double b, double deltaT) {
		return kb * b * (deltaT);
	}
}
