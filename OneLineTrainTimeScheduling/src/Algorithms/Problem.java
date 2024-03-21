package Algorithms;
import java.util.ArrayList;

import Main.SpeedProfile;
import Path.Path;
import Path.Segment;
import Train.Consumption;
import Train.Position;
import Train.State;
import Train.Train;

public class Problem {
	private Train train;
	private Path path;
	private State init;
	private State goal;

	public Problem(Train train, Path path, State init, State goal) {
		this.train = train;
		this.path = path;
		this.init = init;
		this.goal = goal;
	}

	public void print() {
		System.out.println("The train: " + train + " starts from " + init + " and must get to " + goal);
	}

	public SpeedProfile solve() {
//		Initialization
		double currentSpeed = init.getSpeed();
		
		int hSgm = init.getHSgm();
		int tSgm = 0;
		
		double hExit = init.getDist();
		double tExit = path.getLengthOf(0) - (init.getTotalLength() - hExit - train.getLength());
		if(hExit == 0) {
			hSgm++;
			hExit = path.getLengthOf(hSgm);
		}
		
		double currentAcc = train.getMaxAcc();
		double tTotal = 0;
		SpeedProfile sp = new SpeedProfile(init, tTotal);
		
		
		double currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));
		if(currentSpeedLimit == currentSpeed) {
			currentAcc = 0;
		}
		
		Times.getMinT(currentAcc, currentSpeed, tExit, hExit, currentSpeedLimit);
		while(sp.getLastState()!=goal) {
//			Update variables
			double distTraveled = currentSpeed*Times.getTmin() + 0.5*currentAcc*Math.pow(Times.getTmin(), 2);
			currentSpeed += Times.getTmin()*currentAcc;
			hExit -= distTraveled;
			tExit -= distTraveled;
			tTotal += Times.getTmin();
			
			boolean flag = false;
			if(Times.getTmin() == Times.getT2()) {
				System.out.println("t2");
				hSgm++;
				hExit = path.getLengthOf(hSgm);
				currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));

//				Backtracking
				if(currentSpeed > currentSpeedLimit) {
					System.out.println("BackTracking");
//					Algorithm 2
					for(int i = length(sp)-2; i >= 0 && !flag; i--) {
//						Initialization
						double v0 = currentSpeedLimit;
						double v1 = speed(sp, i);
						double v2 = speed(sp, i+1);
						double a = train.getMaxAcc();
						double b = train.getMaxDec();
						
//						Consecutive segments that train does not decelerate
						if(v2 >= v1){
							if(v2 == v1) {
								a = 0;								
							}
							
//							Calculate formulas
							double d1 = dist(sp, i);
							double d = dist(sp);
							double xdec = (Math.pow(v0, 2) - Math.pow(v1, 2) + 2*(b*d+a*d1))/(2*(a+b)); // Deceleration point
							double vHash = Math.sqrt(Math.pow(v0, 2) - 2*b*(xdec-d)); // Starting speed of deceleration
							
							if(xdec>=d1) { //Not in paper
								if(vHash > v1) {
									sp = sp.keepStates(i);
									double t1Hash = (double)((vHash-v1)/a);
									double t2Hash = (double)((vHash-v0)/b);
//									Add 2 new states
									tTotal = sp.getTimeOfState(i);
									tTotal+=t1Hash;
									sp.addState(new State(new Position(path.getSegments(sp.getLastState().getTSgm(), sp.getLastState().getHSgm()), d-xdec), vHash), tTotal);
									tTotal+=t2Hash;
									hExit=path.getLengthOf(hSgm);
									tExit=train.getLength();
									sp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									currentSpeed = v0;
									currentAcc = 0;
									flag = true;
								}
								
								if(vHash == v1) {
									sp = sp.keepStates(i);
									hExit=path.getLengthOf(hSgm);
									tExit=train.getLength();
									if(xdec > d1) {
										double t1Hash = (double)((xdec-d1)/vHash);
										double t2Hash = (double)((vHash-v0)/b);
//										Add 2 new states
										tTotal = sp.getTimeOfState(i);
										tTotal+=t1Hash;
										sp.addState(new State(new Position(path.getSegments(sp.getLastState().getTSgm(), sp.getLastState().getHSgm()), d-xdec), vHash), tTotal);
										tTotal+=t2Hash;
										sp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									}else {
										double tHash = (double)((vHash-v0)/b);
//										Add 1 new state
										tTotal = sp.getTimeOfState(i);
										tTotal+=tHash;
										sp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									}
									currentSpeed = v0;
									currentAcc = 0;
									flag = true;
								}
							}
						}
					}
//					End of Algorithm 2
					if(!flag) {
						return null;
					}
				}
			}
			
			if(Times.getTmin() == Times.getT1()) {
				System.out.println("t1");
				tSgm++;
				tExit = path.getLengthOf(tSgm);
				currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));
				if(currentSpeed < currentSpeedLimit) {
					currentAcc = train.getMaxAcc();
				}
			}
			
			if(Times.getTmin() == Times.getT3() && currentSpeed == currentSpeedLimit) {
				System.out.println("t3");
				currentAcc = 0;
			}
			
			if(!flag) {				
				sp.addState(new State(new Position(path.getSegments(tSgm, hSgm), hExit), currentSpeed), tTotal);
			}
			
/*//		Debugging
			System.out.println("________________________________________");
			sp.printLastState();
			System.out.println("tSgm: " + tSgm);
			System.out.println("hSgm: " + hSgm);
			System.out.println("hExit: " + hExit);
			System.out.println("tExit: " + tExit);
			System.out.println("currentSpeed: " + currentSpeed);
			System.out.println("currentSpeedLimit: " + currentSpeedLimit);
			System.out.println("tTotal: " + tTotal);
*/
			
			if(hSgm == goal.getHSgm() && currentSpeed == goal.getSpeed()) {
				sp.print();
				System.out.println(Consumption.calcConsumtion(sp));
				return sp;
			}
			
			Times.getMinT(currentAcc, currentSpeed, tExit, hExit, currentSpeedLimit);
		}
		return sp;
	}

	public SpeedProfile solveAgain(SpeedProfile oldSp) {
//		Initialization
		double currentSpeed = init.getSpeed();
		
		int hSgm = init.getHSgm();
		int tSgm = init.getTSgm();
		
		double hExit = init.getDist();
		double tExit = path.getLengthOf(hSgm) - (init.getTotalLength() - hExit - train.getLength());
		if(hExit == 0) {
			hSgm++;
			hExit = path.getLengthOf(hSgm);
		}
		
		double currentAcc = train.getMaxAcc();
		double tTotal = oldSp.getEndTimeAtState(oldSp.nOfEntries()-1);;
		SpeedProfile newSp = oldSp.keepStates(oldSp.nOfEntries()-1);
		
		double currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));
		if(currentSpeedLimit == currentSpeed) {
			currentAcc = 0;
		}
		
		Times.getMinT(currentAcc, currentSpeed, tExit, hExit, currentSpeedLimit);
		while(newSp.getLastState()!=goal) {
//			Update variables
			double distTraveled = currentSpeed*Times.getTmin() + 0.5*currentAcc*Math.pow(Times.getTmin(), 2);
			currentSpeed += Times.getTmin()*currentAcc;
			hExit -= distTraveled;
			tExit -= distTraveled;
			tTotal += Times.getTmin();
			
			boolean flag = false;
			if(Times.getTmin() == Times.getT2()) {
				System.out.println("t2");
				hSgm++;
				hExit = path.getLengthOf(hSgm);
				currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));
				
//				Backtracking
				if(currentSpeed > currentSpeedLimit) {
					System.out.println("BackTracking");
//					Algorithm 2
					for(int i = length(newSp)-2; i >= 0 && !flag; i--) {
//						Initialization
						double v0 = currentSpeedLimit;
						double v1 = speed(newSp, i);
						double v2 = speed(newSp, i+1);
						double a = (v2-v1)/(newSp.getEndTimeAtState(i+1)-newSp.getEndTimeAtState(i)); //train.getMaxAcc();
						double b = train.getMaxDec();  
							
//						Calculate formulas
						double d1 = newSp.calcDistTraveled(i);
						double d = dist(newSp);
						double xdec = (Math.pow(v0, 2) - Math.pow(v1, 2) + 2*(b*d+a*d1))/(2*(a+b)); // Deceleration point
						double vHash = Math.sqrt(Math.pow(v0, 2) - 2*b*(xdec-d)); // Starting speed of deceleration
						
						if(xdec>=d1) { //Not in paper
							if(newSp.getState(i+1).equals(init)) {
								newSp = newSp.keepStates(i+1);
								hExit = path.getLengthOf(hSgm);
								tExit = train.getLength();
								if(xdec > d1) {
									double tHash = (double)((vHash-v0)/b);
//									Add 1 new state
									tTotal = newSp.getTimeOfState(i+1);
									tTotal+=tHash;
									newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
								}
								currentSpeed = v0;
								currentAcc = 0;
								flag = true;
							}else {									
								if(vHash > v1) {
									newSp = newSp.keepStates(i);
									double t1Hash = (double)((vHash-v1)/a);
									double t2Hash = (double)((vHash-v0)/b);
//									Add 2 new states
									tTotal = newSp.getTimeOfState(i);
									tTotal+=t1Hash;
									newSp.addState(new State(new Position(path.getSegments(newSp.getLastState().getTSgm(), newSp.getLastState().getHSgm()), d-xdec), vHash), tTotal);
									tTotal+=t2Hash;
									hExit=path.getLengthOf(hSgm);
									tExit=train.getLength();
									newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									currentSpeed = v0;
									currentAcc = 0;
									flag = true;
								}
								
								if(vHash == v1) {
									newSp = newSp.keepStates(i);
									hExit = path.getLengthOf(hSgm);
									tExit = train.getLength();
									if(xdec > d1) {
										double t1Hash = (double)((xdec-d1)/vHash);
										double t2Hash = (double)((vHash-v0)/b);
//										Add 2 new states
										tTotal = newSp.getTimeOfState(i);
										tTotal+=t1Hash;
										newSp.addState(new State(new Position(path.getSegments(newSp.getLastState().getTSgm(), newSp.getLastState().getHSgm()), d-xdec), vHash), tTotal);
										tTotal+=t2Hash;
										newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									}else {
										double tHash = (double)((vHash-v0)/b);
//										Add 1 new state
										tTotal = newSp.getTimeOfState(i);
										tTotal+=tHash;
										newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm), path.getLengthOf(hSgm)), v0), tTotal);
									}
									currentSpeed = v0;
									currentAcc = 0;
									flag = true;
								}
							}
						}
					}
//					End of Algorithm 2
					if(!flag) {
						return null;
					}
				}
			}
			
			if(Times.getTmin() == Times.getT1()) {
				System.out.println("t1");
				tSgm++;
				tExit = path.getLengthOf(tSgm);
				currentSpeedLimit = getSpeedLimit(train, path.getSegments(tSgm, hSgm));
				if(currentSpeed < currentSpeedLimit) {
					currentAcc = train.getMaxAcc();
				}
			}
			
			if(Times.getTmin() == Times.getT3() && currentSpeed == currentSpeedLimit) {
				System.out.println("t3");
				currentAcc = 0;
			}
			
			if(!flag) {				
				newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm), hExit), currentSpeed), tTotal);
			}
			
			if(hSgm == goal.getHSgm() && currentSpeed == goal.getSpeed()) {
				return newSp;
			}
			
			Times.getMinT(currentAcc, currentSpeed, tExit, hExit, currentSpeedLimit);
		}
		return null;
	}

	private double speed(SpeedProfile sp, int i) {
		return sp.getSpeedOf(i);
	}

	private int length(SpeedProfile sp) {
		return sp.nOfEntries();
	}

	private double dist(SpeedProfile sp) {
		State lastState = sp.getLastState();
		int currentHSgm = lastState.getHSgm();
		double distance = calcDist(currentHSgm);
		return distance;
	}
	
	private double dist(SpeedProfile sp, int index) {
		State state = sp.getState(index);
		int currentHSgm = state.getHSgm();
		return calcDist(currentHSgm) - state.getDist();
	}

	private double calcDist(int currentHSgm) {
		double distance = 0;
		for(int i = 0; i <= currentHSgm; i++) { 
			distance += path.getLengthOf(i);
		}
		if(init.getDist() == 0) {
			distance -= path.getLengthOf(init.getHSgm());
		}
		return distance;
	}

	private double getSpeedLimit(Train train, ArrayList<Segment> segments) {
		double lowestSpeedLimit  = segments.get(0).getSpeedLimit();
		for(int i = 1; i < segments.size(); i++) {
			double speedLimit = segments.get(i).getSpeedLimit();
			if(lowestSpeedLimit  > speedLimit) {
				lowestSpeedLimit = speedLimit;
			}
		}
		return Math.min(lowestSpeedLimit, train.getMaxSpeed());
	}
	
	public Path getPath() {
		return path;
	}
	
	static class Times{
		private static double tmin;
		private static double t1;
		private static double t2;
		private static double t3;
		
		public static void getMinT(double currentAcc, double currentSpeed, double tExit, double hExit, double currentSpeedLimit) {
//			Initialization
			t1=1000000000; // Really bit number (Infinite)
			t2=1000000000; // Really bit number (Infinite)
			t3=1000000000; // Really bit number (Infinite)
			
//			Algorithm
			if(currentAcc > 0) {
				t1 = maxRoot(0.5*currentAcc, currentSpeed, -tExit);
				t2 = maxRoot(0.5*currentAcc, currentSpeed, -hExit);
			}else if(currentAcc < 0){
				t1 = maxRoot(0.5*currentAcc, currentSpeed, -tExit);
				t2 = maxRoot(0.5*currentAcc, currentSpeed, -hExit);
			}else {
				t1 = tExit/currentSpeed;
				t2 = hExit/currentSpeed;
			}
			
			if(currentAcc > 0 && currentSpeedLimit > currentSpeed) {
				t3 = (currentSpeedLimit - currentSpeed)/currentAcc;
			}else {
				t3 = 1000000000; // Really bit number (Infinite)
			}
			
			tmin = Math.min(t1, Math.min(t2, t3));
		}

		private static double maxRoot(double a, double b, double c) {
			double delta = (double)Math.pow(b, 2) - 4*a*c;
			if(delta<0) {
				return 1000000000; // Really bit number (Infinite)
			}
			double res = (double)((-b+Math.sqrt(delta))/(2*a));
			return res;
		}

		public static double getTmin() {
			return tmin;
		}

		public static double getT1() {
			return t1;
		}

		public static double getT2() {
			return t2;
		}
		public static double getT3() {
			return t3;
		}
	}

	public State getGoal() {
		return goal;
	}

	public Train getTrain() {
		return train;
	}

	public State getInit() {
		return init;
	}
}
