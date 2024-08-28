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
	private static final double EQUAL_THRESHOLD = 0.0000001;
	
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
			newSp = fuelOptimalSp(newSp, time);
			newSp.print();
			System.out.println("With an extra " + (time-remTime) + " seconds we have cunsumption: " + Consumption.calcConsumtion(newSp));
			System.out.println(newSp.calcDistTraveled(newSp.nOfEntries()-1));
			return newSp;
		}
		System.out.println("The extra time given wasn't valid. It needs to be positive.");
		return givenSp;
	}

	private SpeedProfile fuelOptimalSp(SpeedProfile oldSp, double time) {
		SpeedProfile newSp = oldSp;
		SpeedProfile accToDecSp = oldSp;
		SpeedProfile crSp = oldSp;
		SpeedProfile fuelOptimalSp = oldSp;
		double accToDecTime = 0;
		double crTime = 0;
		while(time>EQUAL_THRESHOLD) {
			System.out.println("f"+ Consumption.calcConsumtion(fuelOptimalSp));
//			Algorithm "Add Cruising State"
			double accToDecTradeOff = -1;
			double tempTime = 0.1;
			if (tempTime > time) {
				tempTime = time;
			}
			newSp = addCruisingStateFavorably(oldSp, tempTime);
			double timeUsedAccToDec = newSp.getEndTimeAtState(newSp.nOfEntries()-1)-oldSp.getEndTimeAtState(oldSp.nOfEntries()-1);
			if(newSp != oldSp) {				
				double newTradeOff = Consumption.calcConsumtion(newSp)/newSp.getEndTimeAtState(newSp.nOfEntries()-1);
//				System.out.println(Consumption.calcConsumtion(newSp));
				if(accToDecTradeOff<newTradeOff) {
					accToDecTradeOff = newTradeOff;
					accToDecSp = newSp;
					accToDecTime = timeUsedAccToDec;
				}
			}
			
//			Algorithm "Slower Constant Speed"
			double timeUsedCr = time;
			Map<Integer, State> crStates = getCruisingStates(oldSp);			
			double crTradeOff = -1;
			for(int index : crStates.keySet()) {
				State crState = oldSp.getState(index);
				double crSpeed = crState.getSpeed();
				
//				Get the state at the start of the acceleration and the state at the start of the cruising
				State targetState = null;
				targetState = oldSp.getState(0); 
				State startCrState = targetState; // The first cruising state
				double tempSpeed = crSpeed;
				for(int i = index-1; i >= 0; i--) {
					double prevSpeed = oldSp.getSpeedOf(i);
					if(EQUAL_THRESHOLD < Math.abs(tempSpeed - prevSpeed)) {
						startCrState = oldSp.getState(i+1);
						if(i==0) {
							break;
						}
						double deltaT = oldSp.getEndTimeAtState(i+1)-oldSp.getEndTimeAtState(i);
						double acc = (tempSpeed-prevSpeed)/deltaT;
						for(int j=i-1; j > 0; j--) {
							deltaT = oldSp.getEndTimeAtState(j+1)-oldSp.getEndTimeAtState(j);
							double tempAcc = (oldSp.getSpeedOf(j+1)-oldSp.getSpeedOf(j))/deltaT;
							if(acc-tempAcc > EQUAL_THRESHOLD) {
								targetState = oldSp.getState(j+1);
								break;
							}
						}
						break;
					}
					tempSpeed = prevSpeed;
				}
				int targetIndex = oldSp.getIndexOf(targetState);
				double targetSpeed = targetState.getSpeed();
				int startCrIndex = oldSp.getIndexOf(startCrState);
				
				double acc = problem.getTrain().getMaxAcc();
				double dec = -problem.getTrain().getMaxDec();
				double newSpeed = crSpeed-0.1;
				if(newSpeed > targetSpeed) {
					if(newSpeed < oldSp.getSpeedOf(index+1)) {
//						newSpeed must not be lower than the speed at the end of the deceleration
						newSpeed = oldSp.getSpeedOf(index+1);
					}
					
//					Duration of instance in olSp
					double oldAccTime = (crSpeed-targetSpeed)/acc;
					double oldCruiseTime = oldSp.getEndTimeAtState(index)-oldSp.getEndTimeAtState(startCrIndex);
					double oldDecTime = (oldSp.getSpeedOf(index+1)-crSpeed)/dec;
					double oldTime = oldAccTime+oldCruiseTime+oldDecTime;
					
					double newAccTime = (newSpeed-targetSpeed)/acc;
					double newDecTime = (oldSp.getSpeedOf(index+1)-newSpeed)/dec;
					
//					Distance covered while cruising at newSpeed
					double totalDistance = newSpeed*(oldAccTime-newAccTime)+0.5*acc*Math.pow(oldAccTime-newAccTime,2)+
							oldSp.calcDistTraveled(index)-oldSp.calcDistTraveled(startCrIndex)+
							crSpeed*(oldDecTime-newDecTime)+0.5*dec*Math.pow(oldDecTime-newDecTime,2);
					
					double newCruiseTime = totalDistance/newSpeed; 
					double newTime = newAccTime+newCruiseTime+newDecTime;
					
					
					timeUsedCr = newTime-oldTime;
					if(timeUsedCr>time) {
//						newSpeed must be higher.
						newSp = slowerCruisingState(oldSp, time); // Calculates newSp utilizing all the time
						timeUsedCr = newSp.getEndTimeAtState(newSp.nOfEntries()-1)-oldSp.getEndTimeAtState(oldSp.nOfEntries()-1);;
					}else {
						newSp = oldSp.keepStates(targetIndex);
						
						double newDist = targetState.getDist()-(targetSpeed*newAccTime+0.5*acc*Math.pow(newAccTime,2));
						if(newDist<0) {
							newDist = path.getLengthOf(targetState.getHSgm())-0.5*acc*Math.pow(newAccTime,2);
						}
//						Add a state at the start of the new cruising (i.e., the end of the acceleration)
						newSp.addState(new State(new Position(path.getSegments(startCrState.getTSgm(), startCrState.getHSgm()),
								newDist), newSpeed), oldSp.getEndTimeAtState(targetIndex) + newAccTime);
						
//						Add a state at the end of the new cruising (i.e., the start of the deceleration)
						newSp.addState(new State(new Position(path.getSegments(crState.getTSgm(), crState.getHSgm()),
								crState.getDist()), newSpeed), oldSp.getEndTimeAtState(targetIndex) + newAccTime + newCruiseTime);
						
						int newIndex = index;
						if(oldSp.getState(index+1).getSpeed() == newSpeed) {
							newIndex++;
						}
//						Append the rest states delayed by the extra available time
						for(int i = newIndex+1; i < oldSp.nOfEntries(); i++) {
							newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+timeUsedCr);								
						}
					}
				}else if(newSpeed <= targetSpeed){
					int newTargetIndex = 0;
					for(int j = targetIndex-1; j >=1; j--) {
						if(oldSp.getSpeedOf(j) != targetSpeed) {
							newTargetIndex = j;
							break;
						}
					}
					State newTargetState = oldSp.getState(newTargetIndex);
					double newTargetSpeed = oldSp.getSpeedOf(newTargetIndex);
					newSpeed = Math.max(targetSpeed, oldSp.getSpeedOf(index+1));
					if(newTargetSpeed > newSpeed) {
						double oldDecTime1 = (targetSpeed-newTargetSpeed)/dec;
						double oldCruiseTime1 = oldSp.getEndTimeAtState(targetIndex)-oldSp.getEndTimeAtState(newTargetIndex+1);
						double oldAccTime = (crSpeed-targetSpeed)/acc;
						double oldCruiseTime2 = oldSp.getEndTimeAtState(index)-oldSp.getEndTimeAtState(startCrIndex);
						double oldDecTime2 = (oldSp.getSpeedOf(index+1)-crSpeed)/dec;
						double oldTime = oldDecTime1+oldCruiseTime1+oldAccTime+oldCruiseTime2+oldDecTime2;
						
						double newDecTime1 = (newSpeed-newTargetSpeed)/dec;
						double newDecTime2 = (oldSp.getSpeedOf(index+1)-newSpeed)/dec;
						
//						Distance covered while cruising at newSpeed
						double totalDistance = newTargetSpeed*(oldDecTime1-newDecTime1)+0.5*dec*Math.pow(oldDecTime1-newDecTime1,2)+
								oldSp.calcDistTraveled(index)-oldSp.calcDistTraveled(newTargetIndex+1)+
								crSpeed*(oldDecTime2-newDecTime2)+0.5*dec*Math.pow(oldDecTime2-newDecTime2,2);
						
						double newCruiseTime = totalDistance/newSpeed; 
						double newTime = newDecTime1+newCruiseTime+newDecTime2;
						
						
						timeUsedCr = newTime-oldTime;
						if(timeUsedCr>time) {
//							newSpeed must be higher.
							newSp = slowerCruisingState(oldSp, time); // Calculates newSp utilizing all the time
							timeUsedCr = newSp.getEndTimeAtState(newSp.nOfEntries()-1)-oldSp.getEndTimeAtState(oldSp.nOfEntries()-1);;
						}else {
							newSp = oldSp.keepStates(newTargetIndex);
							
							double newDist = targetState.getDist()-(targetSpeed*newDecTime1+0.5*acc*Math.pow(newDecTime1,2));
							if(newDist<0) {
								newDist = path.getLengthOf(targetState.getHSgm())-0.5*acc*Math.pow(newDecTime1,2);
							}
//							Add a state at the start of the new cruising (i.e., the end of the acceleration)
							newSp.addState(new State(new Position(path.getSegments(newTargetState.getTSgm(), newTargetState.getHSgm()),
									newDist), newSpeed), oldSp.getEndTimeAtState(newTargetIndex) + newDecTime1);
							
//							Add a state at the end of the new cruising (i.e., the start of the deceleration)
							newSp.addState(new State(new Position(path.getSegments(crState.getTSgm(), crState.getHSgm()),
									crState.getDist()), newSpeed), oldSp.getEndTimeAtState(newTargetIndex) + newDecTime1 + newCruiseTime);
							
							int newIndex = index;
							if(oldSp.getState(index+1).getSpeed() == newSpeed) {
								newIndex++;
							}
//							Append the rest states delayed by the extra available time
							for(int i = newIndex+1; i < oldSp.nOfEntries(); i++) {
								newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+timeUsedCr);								
							}
						}
					}else {
//						Duration of the acceleration and the cruising prior to the instance in the oldSp
						double oldAccTime1 = (targetSpeed-newTargetSpeed)/acc;
						double oldCruiseTime1 = oldSp.getEndTimeAtState(targetIndex) - oldSp.getEndTimeAtState(newTargetIndex+1);
						
//						Duration of instance in olSp
						double oldAccTime2 = (crSpeed-targetSpeed)/acc;
						double oldCruiseTime2 = oldSp.getEndTimeAtState(index)-oldSp.getEndTimeAtState(startCrIndex);
						double oldDecTime = (oldSp.getSpeedOf(index+1)-crSpeed)/dec;
						double oldTime = oldAccTime1+oldCruiseTime1+oldAccTime2+oldCruiseTime2+oldDecTime;
						
						double newAccTime = (newSpeed-newTargetSpeed)/acc;
						double newDecTime = (oldSp.getSpeedOf(index+1)-newSpeed)/dec;
						
//						Distance covered while cruising at newSpeed
						double totalDistance = newSpeed*(oldAccTime1-newAccTime)+0.5*acc*Math.pow(oldAccTime1-newAccTime,2)+
								oldSp.calcDistTraveled(index)-oldSp.calcDistTraveled(newTargetIndex+1)+
								crSpeed*(oldDecTime-newDecTime)+0.5*dec*Math.pow(oldDecTime-newDecTime,2);
						
						
						double newCruiseTime = totalDistance/newSpeed; 
						double newTime = newAccTime+newCruiseTime+newDecTime;
						
						
						timeUsedCr = newTime-oldTime;
						if(timeUsedCr>time) {
//							newSpeed must be higher.
							newSp = slowerCruisingState(oldSp, time); // Calculates newSp utilizing all the time
							timeUsedCr = newSp.getEndTimeAtState(newSp.nOfEntries()-1)-oldSp.getEndTimeAtState(oldSp.nOfEntries()-1);;
						}else {
							newSp = oldSp.keepStates(newTargetIndex);
							
							double newDist = newTargetState.getDist()-(newTargetSpeed*newAccTime+0.5*acc*Math.pow(newAccTime,2));
							if(newDist<0) {
								newDist = path.getLengthOf(newTargetState.getHSgm())-0.5*acc*Math.pow(newAccTime,2);
							}
							
//							Add a state at the start of the new cruising (i.e., the end of the acceleration)
							newSp.addState(new State(new Position(path.getSegments(startCrState.getTSgm(), startCrState.getHSgm()),
									newDist), newSpeed), oldSp.getEndTimeAtState(newTargetIndex) + newAccTime);
							
//							Add a state at the end of the new cruising (i.e., the start of the deceleration)
							newSp.addState(new State(new Position(path.getSegments(crState.getTSgm(), crState.getHSgm()),
									crState.getDist()), newSpeed), oldSp.getEndTimeAtState(newTargetIndex) + newAccTime + newCruiseTime);
							
							int newIndex = index;
							if(oldSp.getState(index+1).getSpeed() == newSpeed) {
								newIndex++;
							}
//							Append the rest states delayed by the extra available time
							for(int i = newIndex+1; i < oldSp.nOfEntries(); i++) {
								newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+timeUsedCr);								
							}
						}
					}
				}
//				Calculate trade-off and store the speed profile if it has higher trade-off 
//				double newTradeOff = Consumption.calcConsumtion(newSp)/newSp.getEndTimeAtState(newSp.nOfEntries()-1);
				double newTradeOff = Consumption.calcConsumtion(newSp)/newSp.getEndTimeAtState(newSp.nOfEntries()-1);
				System.out.println(Consumption.calcConsumtion(newSp));
				if(crTradeOff<newTradeOff) {
					crTradeOff = newTradeOff;
					crSp = newSp;
					crTime = timeUsedCr;
				}
			}
			
//			Update maxSp, oldSp, time and remTime
			if(accToDecTradeOff == -1 && crTradeOff == -1) {
				// No modifications are feasible
				return oldSp;
			}else if(accToDecTradeOff == -1) {
				if(crSp == oldSp) {
					return oldSp;
				}
//				Only "Add Cruising State" technique can be employed.
				fuelOptimalSp = crSp;
				oldSp = fuelOptimalSp;
				remTime -= crTime;
				time -= crTime;
			}else if(crTradeOff == -1) {
				if(accToDecSp == oldSp) {
					return oldSp;
				}
//				Only "Slower Cruising State" technique can be employed.
				fuelOptimalSp = accToDecSp;
				oldSp = fuelOptimalSp;
				remTime -= accToDecTime;
				time -= accToDecTime;
			}else {
				if(crSp == oldSp && accToDecSp == oldSp) {
					return oldSp;
				}
//				Both techniques can be employed. Choose the one with the least trade-off
				if(accToDecTradeOff<crTradeOff) {
					fuelOptimalSp = accToDecSp;
					oldSp = fuelOptimalSp;
					remTime -= accToDecTime;
					time -= accToDecTime;
				}else {
					fuelOptimalSp = crSp;
					oldSp = fuelOptimalSp;
					remTime -= crTime;
					time -= crTime;
				}
			}
		}
		return fuelOptimalSp;
	}

//	Given a cruising state in between of an acceleration and deceleration state, alter it so the locomotive cruises at a lower speed
	private SpeedProfile slowerCruisingState(SpeedProfile oldSp, double time) {
		SpeedProfile crSp = oldSp;
		SpeedProfile newSp = oldSp;
		double timeUsedCr = time;
		Map<Integer, State> crStates = getCruisingStates(oldSp);
		
		double maxTradeOff = -1;
		for(int index : crStates.keySet()) {
			State crState = crStates.get(index);;
			double crSpeed = crState.getSpeed();
				
//			Get the states at the start of the acceleration and the end of cruising 
			State targetState = null;
			targetState = oldSp.getState(0); 
			State startCrState = targetState;
			double tempSpeed = crSpeed;
			for(int i = index-1; i >= 0; i--) {
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
						if(acc-tempAcc > EQUAL_THRESHOLD) {
							targetState = oldSp.getState(j+1);
							break;
						}
					}
					break;
				}
				tempSpeed = prevSpeed;
			}
			int targetIndex = oldSp.getIndexOf(targetState);
			double targetSpeed = targetState.getSpeed();
			int startCrIndex = oldSp.getIndexOf(startCrState);
			
//			Find the speed of the new cruising
			double maxSpeed = crSpeed;
			double minSpeed = Math.max(targetSpeed, oldSp.getSpeedOf(index+1));
			double speed = (maxSpeed+minSpeed)/2; // The cruising speed
			
//		 	Distance traveled until reaching crSpeed during acceleration (in the oldSp)
			double targetStateDistance = oldSp.calcDistTraveled(targetIndex);
			double deltaT = oldSp.getEndTimeAtState(startCrIndex)-oldSp.getEndTimeAtState(targetIndex);
			double acc = (crSpeed-targetSpeed)/(deltaT);			
			double accTime = (speed-targetSpeed)/acc;
			double ds = targetStateDistance + targetSpeed*accTime + 0.5*acc*Math.pow(accTime, 2);
			
//		 	Distance traveled until reaching the end of the instance (in the oldSp)
			deltaT = oldSp.getEndTimeAtState(index+1)-oldSp.getEndTimeAtState(index);
			double dec = (oldSp.getSpeedOf(index+1)-crSpeed)/(deltaT);
			double decTime = (speed-crSpeed)/dec;
			double de = oldSp.calcDistTraveled(index+1);
			
//			Find the crSpeed at which the train travels the same distance utilizing all the extra time
			while(true) {
				deltaT = (oldSp.getEndTimeAtState(index)-oldSp.getEndTimeAtState(startCrIndex))+(crSpeed-speed)/acc+(speed-crSpeed)/dec+timeUsedCr;
				double dTraveled = speed*deltaT + speed*decTime+0.5*dec*Math.pow(decTime, 2); //Distance that can be traveled on new state
				double totalDistance = ds + dTraveled;
				
				if(Math.abs(totalDistance - de) < EQUAL_THRESHOLD) {
					break;
				}
				
				if(totalDistance > de) {
//					That means that we need a higher crSpeed
					maxSpeed = speed;
				}else {
//					That means that we need a lower crSpeed
					minSpeed = speed;					
				}
				
//				Update the variables
				speed = (maxSpeed+minSpeed)/2;
				
				accTime = (speed-targetSpeed)/acc;
				decTime = (oldSp.getSpeedOf(index+1)-speed)/dec;
				ds = targetStateDistance + targetSpeed*accTime + 0.5*acc*Math.pow(accTime, 2);
			}
			newSp = oldSp.keepStates(targetIndex);
			double newDist = targetState.getDist()-(targetSpeed*accTime+0.5*acc*Math.pow(accTime,2));
			if(newDist<0) {
				newDist = path.getLengthOf(targetState.getHSgm())-0.5*acc*Math.pow(accTime,2);
			}
			
//			Add a state at the start of the new cruising (i.e., the end of the acceleration)
			newSp.addState(new State(new Position(path.getSegments(startCrState.getTSgm(), startCrState.getHSgm()),
					newDist), speed), oldSp.getEndTimeAtState(targetIndex) + accTime);
			
			
//			Add a state at the end of the new cruising (i.e., the start of the deceleration)
			newSp.addState(new State(new Position(path.getSegments(crState.getTSgm(), crState.getHSgm()),
					crState.getDist()), speed), oldSp.getEndTimeAtState(targetIndex) + accTime + deltaT);
			
			int newIndex = index;
			if(oldSp.getState(index+1).getSpeed() == speed) {
				newIndex++;
			}
//			Add a state at the end of the new cruising (i.e., the start of the deceleration)
			for(int i = newIndex+1; i < oldSp.nOfEntries(); i++) {
				newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+timeUsedCr);								
			}
			
//			Calculate trade-off and store the speed profile if it has higher trade-off
			double newTradeOff = Consumption.calcConsumtion(newSp)/newSp.getEndTimeAtState(newSp.nOfEntries()-1);
			if(newSp == oldSp) {
				newTradeOff = -1;
			}
 			if(maxTradeOff<newTradeOff) {
				maxTradeOff = newTradeOff;
				crSp = newSp;
			}
		}
		return crSp;
	}
	
	private SpeedProfile addCruisingStateFavorably(SpeedProfile oldSp, double time){
		SpeedProfile newSp = oldSp;
		SpeedProfile accToDecSp = oldSp;
		if(time > EQUAL_THRESHOLD) {
			Map<Integer, State> accToDecStates = getAccToDecStates(oldSp);
			if(accToDecStates.isEmpty()) {
				return oldSp;
			}
			double timeUsedAccToDec = time;
			double accToDecTradeOff =  -1;
			for(int index : accToDecStates.keySet()) {
//			Acquire the state at the start of the acceleration
				State accToDecState = oldSp.getState(index);
				double a = (accToDecState.getSpeed()-oldSp.getSpeedOf(index-1))/(oldSp.getTimeOfState(index)-oldSp.getTimeOfState(index-1));
				State targetState = getTargetStateAcc(oldSp, index, a);
				int targetIndex = oldSp.getIndexOf(targetState);
				
				double peakSpeed = accToDecState.getSpeed();
				
//			Find the speed of the cruising state that will be added
				double maxSpeed = peakSpeed;
				double minSpeed = Math.max(targetState.getSpeed(), oldSp.getSpeedOf(index+1));
				double crSpeed = (maxSpeed+minSpeed)/2; // Cruising speed
				
//			Distance traveled until reaching crSpeed during accelerating (in the oldSp)
				double targetStateDistance = oldSp.calcDistTraveled(targetIndex);
				double deltaT = oldSp.getEndTimeAtState(index)-oldSp.getEndTimeAtState(targetIndex);
				double acc = (peakSpeed-targetState.getSpeed())/(deltaT);			
				double crSpeedTimeAcc = (crSpeed-targetState.getSpeed())/acc;
				double ds = targetStateDistance + targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc, 2);
				
//			Distance traveled until reaching crSpeed during decelerating (in the oldSp)
				double accToDecStateDistance = oldSp.calcDistTraveled(index);
				deltaT = oldSp.getEndTimeAtState(index+1)-oldSp.getEndTimeAtState(index);
				double dec = (oldSp.getSpeedOf(index+1)-peakSpeed)/(deltaT);			
				double crSpeedTimeDec = (crSpeed-peakSpeed)/dec;
				double de = accToDecStateDistance + peakSpeed*crSpeedTimeDec + 0.5*dec*Math.pow(crSpeedTimeDec, 2);
				
//			The sum of the durations of the acceleration and deceleration between peakSpeed and crSpeed (in the oldSp)
				double b = (peakSpeed-crSpeed)/acc + (crSpeed-peakSpeed)/dec;
				
//			Find the crSpeed that verifies the equation:
//			[distance traveled from crSpeed (while acc) to crSpeed (while dec) == distance of cruising at crSpeed with extra time] 
				while(true) {
					double dTraveled = crSpeed*(timeUsedAccToDec+b); // Distance traveled while cruising on crSpeed utilizing extraTimeUsed
					double totalDistance = ds + dTraveled;
					
					if(Math.abs(totalDistance - de) < EQUAL_THRESHOLD) {
						break;
					}
					
					if(maxSpeed == minSpeed) {
						return addCruisingStateFavorably(oldSp, time-0.01);
					}
					
					if(totalDistance > de) {
//					That means that we need a lower crSpeed
						maxSpeed = crSpeed;
					}else {
//					That means that we need a higher crSpeed
						minSpeed = crSpeed;					
					}
					
					
					
//				Update the variables
					crSpeed = (maxSpeed+minSpeed)/2;
					
					crSpeedTimeAcc = (crSpeed-targetState.getSpeed())/acc;
					ds = targetStateDistance + targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc, 2);
					
					crSpeedTimeDec = (crSpeed-peakSpeed)/dec;
					de = accToDecStateDistance + peakSpeed*crSpeedTimeDec + 0.5*dec*Math.pow(crSpeedTimeDec, 2);
					
					b = (peakSpeed-crSpeed)/acc + (crSpeed-peakSpeed)/dec;
				}
				
				newSp = oldSp.keepStates(targetIndex);
				
//			Add a state at the start of cruising. The new end of the acceleration
				double newTimeAcc = oldSp.getEndTimeAtState(targetIndex) + crSpeedTimeAcc;
				double newDistAcc = targetState.getDist() - (targetState.getSpeed()*crSpeedTimeAcc + 0.5*acc*Math.pow(crSpeedTimeAcc,2));
				newSp.addState(new State(new Position(path.getSegments(accToDecState.getTSgm(), accToDecState.getHSgm()),
						newDistAcc), crSpeed), newTimeAcc);
				
//			Add a state at the start of cruising. The new end of the acceleration				
				double newTimeDec = newTimeAcc+timeUsedAccToDec+b;
				double newDistDec = oldSp.calcDistTraveled(index+1)-de;
				
//			Calculate the hSgm and tSgm at the end of the cruising state
				Train train = problem.getTrain();
				int hSgm = oldSp.getState(index+1).getHSgm();
				double sgmLength = path.getLengthOf(hSgm);
				double oldDistDec = oldSp.getState(index+1).getDist();
				if(sgmLength==oldDistDec) {
					hSgm--;
				}
				sgmLength = path.getLengthOf(hSgm);
				double trainLength = train.getLength();
				while(sgmLength < trainLength) {
					hSgm--;
					sgmLength += path.getLengthOf(hSgm);
				}
				int tSgm = hSgm;
				sgmLength = path.getLengthOf(tSgm); 
				while(sgmLength < trainLength) {
					tSgm--;
					sgmLength += path.getLengthOf(tSgm);
				}
				
//			Add a state at the end of cruising. The new start of the deceleration
				newSp.addState(new State(new Position(path.getSegments(tSgm, hSgm),
						newDistDec), crSpeed), newTimeDec);
				
				int newIndex = index;
				if(oldSp.getState(index+1).getSpeed() == crSpeed) {
					newIndex++;
				}
//			Append the rest states delayed by the extra available time utilized
				for(int i = newIndex+1; i < oldSp.nOfEntries(); i++) {
					newSp.addState(oldSp.getState(i), oldSp.getEndTimeAtState(i)+timeUsedAccToDec);								
				}
				
//			Append the rest states delayed by the extra available time utilized
				double newTradeOff = Consumption.calcConsumtion(newSp)/newSp.getEndTimeAtState(newSp.nOfEntries()-1);
				if(accToDecTradeOff<newTradeOff) {
					accToDecTradeOff = newTradeOff;
					accToDecSp = newSp;
				}
			}
		}
		return accToDecSp;
	}

//	Get the state in which the acceleration started (with a fixed acceleration rate)
	private State getTargetStateAcc(SpeedProfile oldSp, int index, double maxAcc) {
		State targetState = oldSp.getState(0);
		double tempAcc = maxAcc;
		for(int i = index-1; i > 0; i--) {
			double prevAcc = (oldSp.getSpeedOf(i)-oldSp.getSpeedOf(i-1))/(oldSp.getTimeOfState(i)-oldSp.getTimeOfState(i-1));
			if(EQUAL_THRESHOLD < tempAcc-prevAcc) {
				return oldSp.getState(i);
			}
		}
		return targetState;
	}

	private Map<Integer, State> getCruisingStates(SpeedProfile oldSp) {
		Map<Integer, State> crStates = new HashMap<Integer, State>();
		
//		Get the states that a deceleration starts
		double startSpeed = oldSp.getState(0).getSpeed();
		for(int i = 1; i < oldSp.nOfEntries()-1; i++) {
			State aState = oldSp.getState(i);
			double endSpeed = aState.getSpeed();
			if(endSpeed == startSpeed) {
//				We want the train to be accelerating before cruising
				boolean prevAcc = false;
				for(int j = i-1; j >= 0; j--) {
					if(oldSp.getSpeedOf(j) < endSpeed) {
						prevAcc = true;
						break;
					}else if(oldSp.getSpeedOf(j) > endSpeed) {
						prevAcc = false;
						break;
					}
				}
				
				double nextSpeed = oldSp.getState(i+1).getSpeed(); 
				if(nextSpeed < endSpeed && prevAcc) {
					crStates.put(i, aState);
				}
			}
			startSpeed = endSpeed;
		}
		return crStates;
	}
	
	private Map<Integer, State> getAccToDecStates(SpeedProfile sp) {
		Map<Integer, State> accToDecStates = new HashMap<Integer, State>();
		
		double startSpeed = sp.getState(0).getSpeed();
		for(int i = 1; i < sp.nOfEntries()-1; i++) {
			State aState = sp.getState(i);
			double endSpeed = aState.getSpeed();
			if(endSpeed > startSpeed) {
				if(sp.getState(i+1).getSpeed() < endSpeed) {
					accToDecStates.put(i, aState);
				}
			}
			startSpeed = endSpeed;
		}
		return accToDecStates;
	}
}
