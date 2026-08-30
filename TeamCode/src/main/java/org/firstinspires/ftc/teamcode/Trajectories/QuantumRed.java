package org.firstinspires.ftc.teamcode.Trajectories;

import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class QuantumRed {
    public static Pose2D[] shootPos = {
            new Pose2D(-1320,370,Math.toRadians(230)),
            new Pose2D(-1470,400,Math.toRadians(290)),
            new Pose2D(-1520,420,Math.toRadians(290)),
            new Pose2D(-1470,400,Math.toRadians(290)),
            new Pose2D(-1470,400,Math.toRadians(290)),
            new Pose2D(-1470,400,Math.toRadians(290)),
            new Pose2D(-1470,400,Math.toRadians(290)),
            new Pose2D(-1470,400,Math.toRadians(290)),
    };
    public static Pose2D gatePos = new Pose2D(-1860, -720, Math.toRadians(245));
    public static Pose2D goingGatePos = new Pose2D(-1850,-600,Math.toRadians(290));
    public static Pose2D afterCollectPos = new Pose2D(-1820,-600,Math.PI*3/2 + 0.175);
    public static Pose2D afterSpike1Pos = new Pose2D(-1500,-647,Math.PI*3/2);
    public static Pose2D afterSpike2Pos = new Pose2D(-1790,-645,Math.PI*3/2);

    public static Pose2D[] spike1Pos = {
            new Pose2D(-1310,-100,Math.PI*3/2),
            new Pose2D(-1310,-580,Math.PI*3/2),
    };
    public static Pose2D[] spike2Pos ={
            new Pose2D(-1920,-100,Math.PI*3/2),
            new Pose2D(-1920,-420,Math.PI*3/2),
    };
}
