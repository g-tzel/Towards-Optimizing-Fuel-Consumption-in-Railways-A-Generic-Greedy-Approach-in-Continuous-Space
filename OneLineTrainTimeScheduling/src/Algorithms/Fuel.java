package Algorithms;

import java.util.HashMap;
import java.util.Map;

import Main.SpeedProfile;
import Path.Path;
import Train.Consumption;
import Train.Position;
import Train.State;
import Train.Train;

public class Fuel {
	private SpeedProfile givenSp;
	private Problem problem;
	private Path path;
	private double remTime = 0;
	
	public Fuel(SpeedProfile sp, Problem problem) {
		this.givenSp=sp;
		this.problem = problem;
		path = problem.getPath();
		System.out.println("Current speed profile has cunsumption: " + Consumption.calcConsumtion(sp));
	}
	
	public SpeedProfile solve(double time) {
		if(time>0) {
			remTime = time;
			SpeedProfile newSp = givenSp;
//			For now, choose on of three
			newSp = replaceCru(newSp, remTime);
			newSp = smoothAcc(newSp, remTime);
			newSp = addCru(newSp, remTime);
			newSp.print();
			System.out.println("With an extra " + (time-remTime) + " seconds we have cunsumption: " + Consumption.calcConsumtion(newSp));
			return newSp;
		}
		System.out.println("Not valid extra time given.");
		return givenSp;
	}

//	Add cruising state for every (if the extra time allows) occasion in which the train accelerates and the immediately decelerates 
	private SpeedProfile addCru(SpeedProfile oldSp, double time) {
		SpeedProfile newSp = oldSp;
		while(time>0) {
			double extraTimeUsed = time; // We may not use all available extra time
			Map<Integer, State> accToDecStates = getAccToDecStates(oldSp);
	//		Get state with highest consumption
			double max = 0;
			int maxIndex = -1;
			for(int index : accToDecStates.keySet()) {
	//			This is the time in which the deceleration starts in the Speed Profile
				double tmin = oldSp.getTimeOfState(index);
				State accToDecState = accToDecStates.get(index);
				double a = (accToDecState.getSpeed()-oldSp.getSpeedOf(index-1))/(tmin-oldSp.getTimeOfState(index-1));
				
				State targetState = getTargetStateAcc(oldSp, index, a);
				int targetIndex = oldSp.getIndexOf(targetState);
				
	//			Calculate consumption of acceleration.
				SpeedProfile tempSp = new SpeedProfile();
				for(int i = targetIndex; i <= index; i++) {
					tempSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i));
				}
				double tempCons = Consumption.calcConsumtion(tempSp);
				if(max<tempCons) {
					max = tempCons;
					maxIndex = index;
				}
			}
			
//			If no more cases to check, end task
			if(maxIndex < 1) {
				return newSp;
			}
			
	//		This is the time in which the deceleration starts in the Speed Profile
			double tmin = oldSp.getTimeOfState(maxIndex);
			
//			Get first the state with different acceleration rate (targetState)
			State accToDecState = accToDecStates.get(maxIndex);
			double a = (accToDecState.getSpeed()-oldSp.getSpeedOf(maxIndex-1))/(tmin-oldSp.getTimeOfState(maxIndex-1));
			State targetState = getTargetStateAcc(oldSp, maxIndex, a);
			int targetIndex = oldSp.getIndexOf(targetState);
			
			
//			Find the speed of the cruising state that will be added
			double maxSpeed = accToDecState.getSpeed();
			double minSpeed = targetState.getSpeed(); // Could be that or the max speed of that and the speed after the deceleration state
			double crSpeed = (maxSpeed+minSpeed)/2; // Cruising speed
			
			// Distance traveled until reaching crSpeed while accelerating (in the oldSp)
			double targetStateDistance = oldSp.calcDistTraveled(targetIndex);
			double deltaT = oldSp.getEndTimeAtState(maxIndex)-oldSp.getEndTimeAtState(targetIndex);
			double acc = (accToDecState.getSpeed()-targetState.getSpeed())/(deltaT);			
			double crSpeedTimeAcc = (crSpeed-targetState.getSpeed())/acc;
			double ds = targetStateDistance + targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc, 2);
			
			// Distance traveled until reaching crSpeed while decelerating (in the oldSp)
			double accToDecStateDistance = oldSp.calcDistTraveled(maxIndex);
			deltaT = oldSp.getEndTimeAtState(maxIndex+1)-oldSp.getEndTimeAtState(maxIndex);
			double dec = (oldSp.getSpeedOf(maxIndex+1)-accToDecState.getSpeed())/(deltaT);			
			double crSpeedTimeDec = (crSpeed-accToDecState.getSpeed())/dec;
			double de = accToDecStateDistance + accToDecState.getSpeed()*crSpeedTimeDec + 0.5*dec*Math.pow(crSpeedTimeDec, 2);
			
			double b; // Time it gets to accelerate to accToDecState.getSpeed() and then decelerate to crSpeed (in the oldSp)
			
//			Find the crSpeed that verifies the equation:
//			[distance from crSpeed (while acc) to crSpeed (while dec) == distance of cruising at crSpeed with extra time] 
			while(true) {
				b = (accToDecState.getSpeed()-crSpeed)/acc + (crSpeed-accToDecState.getSpeed())/dec;
				double dTraveled = crSpeed*(extraTimeUsed+b); //Distance that can be traveled while cruising on crSpeed for extraTimeUsed
				double totalDistance = ds + dTraveled;
				
				if(totalDistance == de) {
					break;
				}
				
				if(totalDistance > de) {
//					That means that it is possible with the current crSpeed, but we won't use all the available extra time
					double temp = (de-ds)/crSpeed - b;
					if (temp < extraTimeUsed) {
						extraTimeUsed = temp;
						break;
					}
					maxSpeed = crSpeed; // Possibly won't ever run. TODO: Check
				}else {
//					That means that we need a higher crSpeed
					minSpeed = crSpeed;					
				}
				
//				Update the variables' values
				crSpeed = (maxSpeed+minSpeed)/2;
				
				crSpeedTimeAcc = (crSpeed-targetState.getSpeed())/acc;
				ds = targetStateDistance + targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc, 2);

				crSpeedTimeDec = (crSpeed-accToDecState.getSpeed())/dec;
				de = accToDecStateDistance + accToDecState.getSpeed()*crSpeedTimeDec + 0.5*dec*Math.pow(crSpeedTimeDec, 2);
			}
			
			newSp = oldSp.keepStates(targetIndex);
			
//			Add a state from which the cruising state will start. The new stop of the acceleration
			double newTimeAcc = oldSp.getEndTimeAtState(targetIndex) + crSpeedTimeAcc;
			double newDistAcc = targetState.getDist() - (targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc,2));
			newSp.addState(new State(new Position(path.getSegments(accToDecState.getTSgm(), accToDecState.getHSgm()),
					newDistAcc), crSpeed), newTimeAcc);
			
//			Add a state at the end of the cruising state. The new start of the deceleration
			double newTimeDec = newTimeAcc+extraTimeUsed+b;
			double newDistDec = accToDecState.getDist() - crSpeed*(extraTimeUsed+b);
			newSp.addState(new State(new Position(path.getSegments(oldSp.getState(maxIndex+1).getTSgm(), oldSp.getState(maxIndex+1).getHSgm()),
					newDistDec), crSpeed), newTimeDec);
			
//			TODO:			
//			TODO: It should recalculate the best route. It's not necessary that the next state is reachable from the new last state. 
//			That's needed only if minSpeed==targetState.getSpeed()
			
//			Create a problem
			State init = newSp.getLastState();
			State goal = problem.getGoal();
			Train train = problem.getTrain();
			Problem problem = new Problem(train, path, init, goal);
//			problem.solveAgain(newSp);
			
//			Add the remaining states with updated time
			for(int i = maxIndex+1; i < oldSp.nOfEntries(); i++) {
				newSp.addState(oldSp.getState(i), oldSp.getTimeOfState(i)+extraTimeUsed);
			}
			oldSp = newSp; // In case there are multiple accToDecStates
			
//			Update available time for other modifications of the Speed Profile
			remTime -= extraTimeUsed;
			time -= extraTimeUsed;
		}
		
		return newSp;
	}

//	Replace cruising states that follow an accelerating state
	private SpeedProfile replaceCru(SpeedProfile oldSp, double time) {
		SpeedProfile newSp = oldSp;
		while(time>0) {
			double extraTimeUsed = time;
			Map<Integer, State> crStates = crStates(oldSp);
			double max = 0;
			int maxIndex = -1;
			for(int index : crStates.keySet()) {
//				This is the time in which the deceleration starts in the Speed Profile
				State crState = crStates.get(index);
				double crSpeed = crState.getSpeed();
				
//				Get the first accelerating state of the states before the cruising state
				State targetState = null;
				targetState = oldSp.getState(0); 
				double tempSpeed = crSpeed;
				for(int i = index-1; i >= 0; i--) {
					double prevSpeed = oldSp.getSpeedOf(i);
					if(prevSpeed < tempSpeed) {
						if(i==0) {
							break;
						}
						double deltaT = oldSp.getEndTimeAtState(i+1)-oldSp.getEndTimeAtState(i);
						double acc = (tempSpeed-prevSpeed)/deltaT;
						for(int j=i-1; j > 0; j--) {
							deltaT = oldSp.getEndTimeAtState(j+1)-oldSp.getEndTimeAtState(j);
							double tempAcc = (oldSp.getSpeedOf(j+1)-oldSp.getSpeedOf(j))/deltaT;
							if(acc > tempAcc) {
								targetState = oldSp.getState(j+1);
								break;
							}
						}
						break;
					}
					tempSpeed = prevSpeed;
				}
				int targetIndex = oldSp.getIndexOf(targetState);
				
//				Calculate consumption of acceleration.
				SpeedProfile tempSp = new SpeedProfile();
				for(int i = targetIndex; i <= index; i++) {
					tempSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i));
				}
				double tempCons = Consumption.calcConsumtion(tempSp);
				if(max<tempCons) {
					max = tempCons;
					maxIndex = index;
				}
			}
			
			if(maxIndex < 1) {
				return oldSp;
			}
			State crState = oldSp.getState(maxIndex);
			double crSpeed = crState.getSpeed();
				
			State targetState = null;
//			Get the first accelerating state of the states before the cruising state
			targetState = oldSp.getState(0); 
			State startCrState = targetState; // The first cruising state
			double tempSpeed = crSpeed;
			for(int i = maxIndex-1; i >= 0; i--) {
				double prevSpeed = oldSp.getSpeedOf(i);
				if(prevSpeed < tempSpeed) {
					startCrState = oldSp.getState(i+1);
					if(i==0) {
						break;
					}
					double deltaT = oldSp.getEndTimeAtState(i+1)-oldSp.getEndTimeAtState(i);
					double acc = (tempSpeed-prevSpeed)/deltaT;
					for(int j=i-1; j > 0; j--) {
						deltaT = oldSp.getEndTimeAtState(j+1)-oldSp.getEndTimeAtState(j);
						double tempAcc = (oldSp.getSpeedOf(j+1)-oldSp.getSpeedOf(j))/deltaT;
						if(acc > tempAcc) {
							targetState = oldSp.getState(j+1);
							break;
						}
					}
					break;
				}
				tempSpeed = prevSpeed;
			}
			int targetIndex = oldSp.getIndexOf(targetState);
			int startCrIndex = oldSp.getIndexOf(startCrState);
			
//			Find the speed and the duration of the new acceleration state that will be added
			double maxSpeed = crState.getSpeed();
			double minSpeed = targetState.getSpeed();
			double speed = (maxSpeed+minSpeed)/2; // The speed in which the acceleration rate will change
			
// 			Distance traveled until reaching speed (in the oldSp)
			double targetStateDistance = oldSp.calcDistTraveled(targetIndex);
			double deltaT = oldSp.getEndTimeAtState(startCrIndex)-oldSp.getEndTimeAtState(targetIndex);
			double oldAcc = (crSpeed-targetState.getSpeed())/(deltaT);			
			double accTime = (speed-targetState.getSpeed())/oldAcc;
			double ds = targetStateDistance + targetState.getSpeed()*accTime + 0.5*oldAcc*Math.pow(accTime, 2);
			
// 			Distance traveled until reaching the end of the cruising state (in the oldSp)
			double de = oldSp.calcDistTraveled(maxIndex);
			double newAcc; // The new acceleration rate
			
//			Find the speed and the acceleration rate so that the train travels the same distance utilizing (some of) the extra time
			while(true) {
				deltaT = oldSp.getEndTimeAtState(maxIndex)-(oldSp.getEndTimeAtState(targetIndex)+accTime)+extraTimeUsed;
				newAcc = (crSpeed-speed)/deltaT;
				double dTraveled = speed*(deltaT)+0.5*newAcc*Math.pow(deltaT,2); //Distance that can be traveled on new state
				double totalDistance = ds + dTraveled;
				
				if(totalDistance == de) {
					break;
				}
				
				if(totalDistance > de) {
//					That means that it is possible with the current speed and acceleration rate, but we won't use all the available extra time
					double temp = (-speed + Math.sqrt(Math.pow(speed, 2) - 4*0.5*newAcc*(-(de-ds))))/(2*0.5*newAcc);
					if (temp < deltaT) {
						extraTimeUsed = temp-(oldSp.getEndTimeAtState(maxIndex)-(oldSp.getEndTimeAtState(targetIndex)+accTime));
					}
					maxSpeed = speed; // Possibly won't ever run. TODO: Check
				}else {
//					That means that we need a higher speed
					minSpeed = speed;					
				}
				
//				Update the variables' values
				speed = (maxSpeed+minSpeed)/2;
				
				accTime = (speed-targetState.getSpeed())/oldAcc;
				ds = targetStateDistance + targetState.getSpeed()*accTime + 0.5*oldAcc*Math.pow(accTime, 2);
			}
			
			newSp = oldSp.keepStates(targetIndex);
//			Add a state with speed. The start of the new state/acceleration rate
			double newDist = targetState.getDist()-(targetState.getSpeed()*accTime+0.5*oldAcc*Math.pow(accTime,2));
			if(newDist<0) {
				newDist = path.getLengthOf(targetState.getHSgm())-0.5*oldAcc*Math.pow(accTime,2);
			}
			newSp.addState(new State(new Position(path.getSegments(startCrState.getTSgm(), startCrState.getHSgm()),
					newDist), speed), oldSp.getEndTimeAtState(targetIndex) + accTime);

//			Add acceleration state with new rate replacing existing acceleration and cruising states
			newSp.addState(new State(new Position(path.getSegments(crState.getTSgm(), crState.getHSgm()),
					crState.getDist()), crSpeed), oldSp.getEndTimeAtState(maxIndex) + extraTimeUsed);
			for(int i = maxIndex+1; i < oldSp.nOfEntries(); i++) {
				newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+extraTimeUsed);
			}
			oldSp = newSp; // In case there are multiple accToDecStates
			
//			Update available time for other modifications of the Speed Profile
			remTime -= extraTimeUsed;
			time -= extraTimeUsed;
		}
		return newSp;
	}

//	Smooth the acceleration (accelerate at a slower rate)
	private SpeedProfile smoothAcc(SpeedProfile oldSp, double time) {		
		SpeedProfile newSp = oldSp;
		while(remTime>0) {
			double extraTimeUsed = time; // TODO: Check if it's worth to use extraTimeUsed=time/accToDecStates.size();
			Map<Integer, State> accToDecStates = getAccToDecStates(oldSp);
//			Get state with highest consumption
			double max = 0;
			int maxIndex = -1;
			for(int index : accToDecStates.keySet()) {
//				This is the time in which the deceleration starts in the Speed Profile
				double tmin = oldSp.getTimeOfState(index);
				State accToDecState = accToDecStates.get(index);
				double a = (accToDecState.getSpeed()-oldSp.getSpeedOf(index-1))/(tmin-oldSp.getTimeOfState(index-1));
				
				State targetState = getTargetStateAcc(oldSp, index, a);
				int targetIndex = oldSp.getIndexOf(targetState);
				
//				Calculate consumption of acceleration.
				SpeedProfile tempSp = new SpeedProfile();
				for(int i = targetIndex; i <= index; i++) {
					tempSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i));
				}
				double tempCons = Consumption.calcConsumtion(tempSp);
				if(max<tempCons) {
					max = tempCons;
					maxIndex = index;
				}
			}
			
//			If no more cases to check, end task
			if(maxIndex < 1) {
				return newSp;
			}

//			This is the time in which the deceleration starts in the Speed Profile
			double tmin = oldSp.getTimeOfState(maxIndex);
			State accToDecState = accToDecStates.get(maxIndex);
			double a = (accToDecState.getSpeed()-oldSp.getSpeedOf(maxIndex-1))/(tmin-oldSp.getTimeOfState(maxIndex-1));
			State targetState = getTargetStateAcc(oldSp, maxIndex, a);
			int targetIndex = oldSp.getIndexOf(targetState);

//			Replace the accToDecState with one of slower acceleration rate
			newSp = oldSp.keepStates(targetIndex);
			newSp.addState(new State(new Position(path.getSegments(accToDecState.getTSgm(), accToDecState.getHSgm()),
					accToDecState.getDist()), accToDecState.getSpeed()), tmin+extraTimeUsed);
			
//			Add the remaining states with updated time
			for(int i = maxIndex+1; i < oldSp.nOfEntries(); i++) {
				newSp.addState(oldSp.getState(i), oldSp.getTimeOfState(i)+extraTimeUsed);
			}
			oldSp = newSp; // In case there are multiple accToDecStates
			
//			Update available time for other modifications of the Speed Profile
			remTime -= extraTimeUsed;
			time -= extraTimeUsed;
		}
		return newSp;
	}

//	Get the state in which the acceleration started (with a fixed acceleration rate)
	private State getTargetStateAcc(SpeedProfile oldSp, int index, double maxAcc) {
		State targetState = oldSp.getState(0);
		double tempAcc = maxAcc;
		for(int i = index-1; i > 0; i--) {
			double prevAcc = (oldSp.getSpeedOf(i)-oldSp.getSpeedOf(i-1))/(oldSp.getTimeOfState(i)-oldSp.getTimeOfState(i-1));
			if(prevAcc < tempAcc) {
				return oldSp.getState(i);
			}
		}
		return targetState;
	}

	private Map<Integer, State> crStates(SpeedProfile oldSp) {
		Map<Integer, State> cruStates = new HashMap<Integer, State>();
		
//		Get the states that a deceleration starts
		double startSpeed = oldSp.getState(0).getSpeed();
		for(int i = 1; i < oldSp.nOfEntries(); i++) {
			State aState = oldSp.getState(i);
			double endSpeed = aState.getSpeed();
			if(endSpeed == startSpeed) {
				if(i+1<oldSp.nOfEntries()) {
					State crState = oldSp.getState(i);
					double crSpeed = crState.getSpeed();
					
//					We want the train to be accelerating before cruising
					boolean prevAcc = false;
					for(int j = i-1; j >= 0; j--) {
						if(oldSp.getSpeedOf(j) < crSpeed) {
							prevAcc = true;
							break;
						}else if(oldSp.getSpeedOf(j) > crSpeed) {
							prevAcc = false;
							break;
						}
					}
					if((oldSp.getState(i+1).getSpeed() < endSpeed || oldSp.getState(i+1).getSpeed() > endSpeed) && prevAcc) {
						cruStates.put(i, aState);
					}
				}
			}
			startSpeed = endSpeed;
		}
		return cruStates;
	}
	
	private Map<Integer, State> getAccToDecStates(SpeedProfile sp) {
		Map<Integer, State> accToDecStates = new HashMap<Integer, State>();
		
		double startSpeed = sp.getState(0).getSpeed();
		for(int i = 1; i < sp.nOfEntries(); i++) {
			State aState = sp.getState(i);
			double endSpeed = aState.getSpeed();
			if(endSpeed > startSpeed) {
				if(i+1<sp.nOfEntries()) {
					if(sp.getState(i+1).getSpeed() < endSpeed) {
						accToDecStates.put(i, aState);
					}
				}
			}
			startSpeed = endSpeed;
		}
		return accToDecStates;
	}
}
