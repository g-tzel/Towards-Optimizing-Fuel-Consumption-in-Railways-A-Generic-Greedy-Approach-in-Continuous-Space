package Main;

import Algorithms.Fuel;
import Algorithms.Problem;
import Path.Path;
import Path.Segment;
import Train.Position;
import Train.State;
import Train.Train;

public class Main {

	public static void main(String[] args) {
//		Run only one at a time
//		problem1();
//		problem2();
//		problem3();
//		problem4();
		problem5();
//		problem6();
//		problem7();
	}

	//	This is a complete problem with short duration
	public static void problem1() {
//		Create a path
		Path p = new Path();
		Segment s0 = new Segment(0, 150, 150, 20);
		Segment s1 = new Segment(150, 300, 150, 20);
		Segment s2 = new Segment(300, 1100, 800, 50);
		Segment s3 = new Segment(1100, 1800, 700, 65);
		Segment s4 = new Segment(1800, 2400, 600, 85);
		Segment sf = new Segment(2400, 2550, 150, 10);
		Segment sfx = new Segment(2550, 2550, 0, 0);
		p.addSegment(s0);
		p.addSegment(s1);
		p.addSegment(s2);
		p.addSegment(s3);
		p.addSegment(s4);
		p.addSegment(sf);
		p.addSegment(sfx);
		
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 78;
		double maxAcc = 1.5;
		double maxDec = 0.5;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(s0, 0), 0);
		State goal = new State(new Position(sfx, 0), 0);
//		State init = new State(new Position(s0, 150), 0);
//		State goal = new State(new Position(sfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(!sp.equals(null)) {
			System.out.println("Has solution");
		}else {
			System.out.println("Doesn't have solution");
		}
		
		
		Fuel f = new Fuel(sp, problem);
		f.solve(10);
	}
	
//	This is a complete problem with average duration
	public static void problem2() {
//		Create a path
		Path p = new Path();
		Segment s0 = new Segment(0, 150, 150, 20);
		Segment s1 = new Segment(150, 300, 150, 40);
		Segment s2 = new Segment(300, 1000, 700, 50);
		Segment s3 = new Segment(1000, 1800, 800, 80);
		Segment s4 = new Segment(1800, 2800, 1000, 85);
		Segment s5 = new Segment(2800, 3000, 200, 40);
		Segment s6 = new Segment(3000, 3300, 300, 60);
		Segment s7 = new Segment(3300, 4000, 700, 50);
		Segment sf = new Segment(4000, 4150, 150, 30);
		Segment sfx = new Segment(4150, 4150, 0, 0);
		p.addSegment(s0);
		p.addSegment(s1);
		p.addSegment(s2);
		p.addSegment(s3);
		p.addSegment(s4);
		p.addSegment(s5);
		p.addSegment(s6);
		p.addSegment(s7);
		p.addSegment(sf);
		p.addSegment(sfx);	
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(s0, 150), 0);
		State goal = new State(new Position(sfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(!sp.equals(null)) {
			System.out.println("Has solution");
		}else {
			System.out.println("Doesn't have solution");
		}
		
		
		Fuel f = new Fuel(sp, problem);
		f.solve(25);
	}
	
//	This is a test problem with long duration and practically no speed limit
	private static void problem3() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 1000, 1000, 100);
		Segment ex1 = new Segment(1000, 10000, 9000, 100);
		Segment exfx = new Segment(10000, 10000, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 1000), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(25);
		}else {
			System.out.println("Doesn't have solution");
		}
	}

//	This is a test problem with average duration and practically no speed limit
	private static void problem4() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 1000, 1000, 100);
		Segment ex1 = new Segment(1000, 4687.5, 3687.5, 100);
		Segment exfx = new Segment(4687.5, 4687.5, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 850), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(25);
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
//	This is a test problem with long duration and a mean speed limit between two segments with practically no speed limit
	private static void problem5() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 1000, 1000, 100);
		Segment ex1 = new Segment(1000, 4025, 3025, 100);
		Segment ex2 = new Segment(4025, 7000, 2975, 50);
		Segment ex3 = new Segment(7000, 15000, 8000, 100);
		Segment exfx = new Segment(15000, 15000, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(ex2);
		p.addSegment(ex3);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 850), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(10);
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
//	This is a test problem with long duration and a low speed limit between two segments with practically no speed limit
	private static void problem6() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 3570.833333, 3570.833333, 65);
		Segment ex1 = new Segment(3570.833333, 5112.5, 1541.666667, 50);
		Segment ex2 = new Segment(5112.5, 13937.17949, 8824.67949, 100);
		Segment exfx = new Segment(13937.17949, 13937.17949, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(ex2);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 3570.833333), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(10);
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
//	This is a test problem with long duration and a low speed limit between two segments with practically no speed limit
	private static void problem7() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 150, 150, 20);
		Segment ex1 = new Segment(150, 300, 150, 40);
		Segment ex2 = new Segment(300, 1000, 700, 50);
		Segment ex3 = new Segment(1000, 1800, 800, 40);
		Segment ex4 = new Segment(1800, 2800, 1000, 40);
		Segment ex5 = new Segment(2800, 3000, 200, 40);
		Segment ex6 = new Segment(3000, 3300, 300, 40);
		Segment exfx = new Segment(3300, 3300, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(ex2);
		p.addSegment(ex3);
		p.addSegment(ex4);
		p.addSegment(ex5);
		p.addSegment(ex6);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1.5;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 150), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(10);
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
}
