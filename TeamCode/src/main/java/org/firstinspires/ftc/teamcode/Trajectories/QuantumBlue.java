package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class QuantumBlue {
    public static Pose2D[] shootPos = {
            new Pose2D(-1320,-360,Math.toRadians(130)),
            new Pose2D(-1520,-415,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
            new Pose2D(-1480,-400,Math.toRadians(75)),
    };
    public static Pose2D gatePos = new Pose2D(-1840, 720, Math.toRadians(117));
    public static Pose2D goingGatePos = new Pose2D(-1830,580,Math.toRadians(75));
    public static Pose2D afterCollectPos = new Pose2D(-1800,620,Math.PI/2 - 0.1);
    public static Pose2D afterSpike1Pos = new Pose2D(-1500,647,Math.PI/2);
    public static Pose2D afterSpike2Pos = new Pose2D(-1780,650,Math.PI/2);
    public static Pose2D[] spike1Pos = {
            new Pose2D(-1290,50,Math.PI/2),
            new Pose2D(-1290,420,Math.PI/2),
    };
    public static Pose2D[] spike2Pos ={
            new Pose2D(-1920,50,Math.PI/2),
            new Pose2D(-1920,420,Math.PI/2),
    };
}
