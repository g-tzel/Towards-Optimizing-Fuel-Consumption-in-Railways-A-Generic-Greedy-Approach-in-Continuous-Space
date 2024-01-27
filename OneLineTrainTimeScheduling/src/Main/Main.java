package Main;

import Path.Path;
import Path.Segment;
import Train.Consumption;
import Train.Position;
import Train.State;
import Train.Train;

public class Main {

	public static void main(String[] args) {
//		problem1();
		problem2();
	}
	
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
		boolean solvable = problem.solve();
		if(solvable) {
			System.out.println("Has solution");
		}else {
			System.out.println("Doesn't have solution");
		}
	}
	
	public static void problem2() {
//		Create a path
		Path p2 = new Path();
		Segment s20 = new Segment(0, 150, 150, 20);
		Segment s21 = new Segment(150, 300, 150, 40);
		Segment s22 = new Segment(300, 1000, 700, 50);
		Segment s23 = new Segment(1000, 1800, 800, 80);
		Segment s24 = new Segment(1800, 2800, 1000, 85);
		Segment s25 = new Segment(2800, 3000, 200, 40);
		Segment s26 = new Segment(3000, 3300, 300, 60);
		Segment s27 = new Segment(3300, 4000, 700, 50);
		Segment s2f = new Segment(4000, 4150, 150, 30);
		Segment s2fx = new Segment(4150, 4150, 0, 0);
		p2.addSegment(s20);
		p2.addSegment(s21);
		p2.addSegment(s22);
		p2.addSegment(s23);
		p2.addSegment(s24);
		p2.addSegment(s25);
		p2.addSegment(s26);
		p2.addSegment(s27);
		p2.addSegment(s2f);
		p2.addSegment(s2fx);	
		p2.print();
		
//		Create a train
		double length2 = 150;
		double maxSpeed2 = 75;
		double maxAcc2 = 1.5;
		double maxDec2 = 1;
		Train t2 = new Train(length2, maxSpeed2, maxAcc2, maxDec2);
		
		t2.print();
		
//		Create a problem
		State init2 = new State(new Position(s20, 150), 0);
		State goal2 = new State(new Position(s2fx, 0), 0);
		Problem problem2 = new Problem(t2, p2, init2, goal2);
		
		problem2.print();
		problem2.solve();
	}
}
