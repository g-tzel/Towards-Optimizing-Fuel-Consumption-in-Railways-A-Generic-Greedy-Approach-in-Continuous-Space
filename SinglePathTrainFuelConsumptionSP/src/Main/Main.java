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
//		problem5();
//		problem6();
//		problem7();
//		problem8();
//		problem9();
//		problem10();
//		problem11();
//		problem12();
//		problem13();
//		problem14();
		problem15();
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
		Segment s0 = new Segment(0, 300, 300, 20);
		Segment s1 = new Segment(300, 450, 150, 40);
		Segment s2 = new Segment(450, 1150, 700, 50);
		Segment s3 = new Segment(1150, 1950, 800, 80);
		Segment s4 = new Segment(1950, 2950, 1000, 85);
		Segment s5 = new Segment(2950, 3150, 200, 40);
		Segment s6 = new Segment(3150, 3450, 300, 60);
		Segment s7 = new Segment(3450, 4150, 700, 50);
		Segment sf = new Segment(4150, 4300, 150, 30);
		Segment sfx = new Segment(4300, 4300, 0, 0);
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
		f.solve(10); //129.52440080825878
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
		Segment ex0 = new Segment(0, 1150, 1150, 100);
		Segment ex1 = new Segment(1150, 4175, 3025, 76);
		Segment ex2 = new Segment(4175, 7150, 2975, 50);
		Segment ex3 = new Segment(7150, 15150, 8000, 100);
		Segment exfx = new Segment(15150, 15150, 0, 0);
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
	
//	This is a test problem with long duration and a low speed limit between two segments with practically no speed limit
	private static void problem6() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 3720.833333, 3720.833333, 65);
		Segment ex1 = new Segment(3720.833333, 5262.5, 1541.666667, 50);
		Segment ex2 = new Segment(5262.5, 10766.666666, 5504.166666, 92);
		Segment exfx = new Segment(10766.666666, 10766.666666, 0, 0);
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
			f.solve(20);
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
		State init = new State(new Position(ex0, 0), 0);
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

	private static void problem8() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 1000, 1000, 50);
		Segment ex1 = new Segment(1000, 1400, 400, 50);
		Segment ex2 = new Segment(1400, 2450, 1050, 50);
		Segment exfx = new Segment(2450, 2450, 0, 10);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(ex2);
		p.addSegment(exfx);
		p.print();
		
//		Create a train
		double length = 150;
		double maxSpeed = 75;
		double maxAcc = 1;
		double maxDec = 1;
		Train t = new Train(length, maxSpeed, maxAcc, maxDec);
		
		t.print();
		
//		Create a problem
		State init = new State(new Position(ex0, 850), 0);
		State goal = new State(new Position(exfx, 0), 10);
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
	
	private static void problem9() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 1000, 1000, 50);
		Segment ex1 = new Segment(1000, 1500, 500, 20);
		Segment ex2 = new Segment(1500, 2500, 1000, 20);
		Segment exfx = new Segment(2500, 2500, 0, 0);
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
	
	private static void problem10() {
//		Create a path
		Path p = new Path();
		Segment s0 = new Segment(0, 500, 500, 20);
		Segment s1 = new Segment(500, 1500, 1000, 20);
		Segment sfx = new Segment(1500, 1500, 0, 0);
		p.addSegment(s0);
		p.addSegment(s1);
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
		State init = new State(new Position(s0, 350), 20);
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
	
	private static void problem11() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 150, 150, 20);
		Segment ex1 = new Segment(150, 300, 150, 20);
		Segment ex2 = new Segment(300, 1000, 700, 80);
		Segment ex3 = new Segment(1000, 1800, 800, 40);
		Segment ex4 = new Segment(1800, 2800, 1000, 80);
		Segment ex5 = new Segment(2800, 3000, 200, 80);
		Segment ex6 = new Segment(3000, 3300, 300, 100);
		Segment exfx = new Segment(3300, 3300, 0, 100);
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
		State init = new State(new Position(ex0, 0), 0);
		State goal = new State(new Position(exfx, 0), 75);
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
	private static void problem12() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 5150, 5150, 100);
		Segment ex1 = new Segment(5150, 6150, 1000, 50);
		Segment ex2 = new Segment(6150, 10150, 4000, 55);
		Segment exfx = new Segment(10150, 10150, 0, 0);
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
		State init = new State(new Position(ex0, 5000), 0);
		State goal = new State(new Position(exfx, 0), 0);
		Problem problem = new Problem(t, p, init, goal);
		
		problem.print();
		SpeedProfile sp = problem.solve();
		if(sp != null) {
			System.out.println("Has solution");
			Fuel f = new Fuel(sp, problem);
			f.solve(15);
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
//	This is a complete problem with average duration
	public static void problem13() {
//		Create a path
		Path p = new Path();
		Segment s0 = new Segment(0, 300, 300, 20);
		Segment s1 = new Segment(300, 450, 150, 40);
		Segment s2 = new Segment(450, 1150, 700, 50);
		Segment s3 = new Segment(1150, 1950, 800, 80);
		Segment s4 = new Segment(1950, 2950, 1000, 85);
		Segment s5 = new Segment(2950, 3150, 200, 40);
		Segment s6 = new Segment(3150, 3450, 300, 35);
		Segment s7 = new Segment(3450, 3650, 200, 30);
		Segment s8 = new Segment(3650, 4300, 650, 70);
		Segment s9 = new Segment(4300, 4450, 150, 55);
		Segment s10 = new Segment(4450, 4650, 200, 65);
		Segment sf = new Segment(4650, 5250, 600, 20);
		Segment sfx = new Segment(5250, 5250, 0, 0);
		p.addSegment(s0);
		p.addSegment(s1);
		p.addSegment(s2);
		p.addSegment(s3);
		p.addSegment(s4);
		p.addSegment(s5);
		p.addSegment(s6);
		p.addSegment(s7);
		p.addSegment(s8);
		p.addSegment(s9);
		p.addSegment(s10);
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
		f.solve(10); //130
	}
	
	private static void problem14() {
//		Create a path
		Path p = new Path();
		Segment ex0 = new Segment(0, 300, 300, 20);
		Segment ex1 = new Segment(300, 450, 150, 20);
		Segment ex2 = new Segment(450, 1150, 700, 80);
		Segment ex3 = new Segment(1150, 1950, 800, 40);
		Segment ex4 = new Segment(1950, 2950, 1000, 85);
		Segment ex5 = new Segment(2950, 3150, 200, 80);
		Segment ex6 = new Segment(3150, 3750, 600, 100);
		Segment exf = new Segment(3750, 3950, 200, 20);
		Segment exfx = new Segment(3950, 3950, 0, 0);
		p.addSegment(ex0);
		p.addSegment(ex1);
		p.addSegment(ex2);
		p.addSegment(ex3);
		p.addSegment(ex4);
		p.addSegment(ex5);
		p.addSegment(ex6);
		p.addSegment(exf);
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
	
//	Paper Problem
	private static void problem15() {
//		Create a path
		Path p = new Path();
		Segment s0 = new Segment(0, 3000, 3000, 60);
		Segment s1 = new Segment(3000, 3250, 250, 50);
		Segment s2 = new Segment(3250, 3450, 200, 83);
		Segment s3 = new Segment(3450, 4950, 1500, 73);
		Segment s4 = new Segment(4950, 5100, 150, 45);
		Segment s5 = new Segment(5100, 5300, 200, 70);
		Segment s6 = new Segment(5300, 5400, 100, 40);
		Segment s7 = new Segment(5400, 6000, 550, 45);
		Segment s8 = new Segment(6000, 6600, 600, 53);
		Segment s9 = new Segment(6600, 6850, 250, 40);
		Segment s10 = new Segment(6850, 8850, 2000, 60);
		Segment sf = new Segment(8850, 9050, 200, 20);
		Segment sfx = new Segment(9050, 9050, 0, 0);
		p.addSegment(s0);
		p.addSegment(s1);
		p.addSegment(s2);
		p.addSegment(s3);
		p.addSegment(s4);
		p.addSegment(s5);
		p.addSegment(s6);
		p.addSegment(s7);
		p.addSegment(s8);
		p.addSegment(s9);
		p.addSegment(s10);
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
		State init = new State(new Position(s0, 2850), 0);
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
		f.solve(10); //130
	}
}

