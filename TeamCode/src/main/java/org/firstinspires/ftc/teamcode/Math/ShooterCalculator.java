package org.firstinspires.ftc.teamcode.Math;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class ShooterCalculator {

    // --- Your Existing Flywheel Control Constants ---
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0.15;
    public static double Kv = 0.00044;
    public static double Ka = 0.00635;
    public static double regression = 0.00045;
    public static double g = 9810.0;
    public static double goalHeight = 67.5;
    public static double passThroughAngle = -Math.PI/6;
    public static double maxHoodAngle = Math.PI/3;
    public static double minHoodAngle = Math.PI/6;
    public static double maxHoodPos = 0.496;
    public static double minHoodPos = 0.11;

    public static double targetAngle = 0;
    public static double hoodPos = 0.5;
    public static double vel = 1200;

    public static void updateTrajectory(double robotX, double robotY, double velX, double velY, double goalX, double goalY) {
        double distance = Math.hypot(goalX - robotX, goalY - robotY);
        double gAngle = Math.atan2(goalY - robotY, goalX - robotX);

        double alpha = Math.atan((2 * goalHeight / distance) - Math.tan(passThroughAngle));
        double v0 = Math.sqrt(Math.abs((g * Math.pow(distance, 2)) / (2 * Math.pow(Math.cos(alpha), 2)
                * (distance * Math.tan(Math.cos(alpha)) - goalHeight))));

        double vAngle = Math.atan2(velY, velX);

        double rAngle = vAngle - gAngle;
        double vRadial = -Math.cos(rAngle) * Odo.avgVel();
        double vTan = Math.sin(rAngle) * Odo.avgVel();

        double time = distance / (v0 * Math.cos(alpha));

        double vCompensation = (distance / time) + vRadial;
        double vx = Math.hypot(vCompensation, vTan);
        double vy = v0 * Math.sin(alpha);

        double newAlpha = Math.atan2(vy, vx);
        double velocity = Math.hypot(vx, vy);

        double turretOffset = Math.atan2(vTan, vCompensation);

        targetAngle = gAngle + turretOffset;
        hoodPos = FromRadsToPos(Math.clamp(newAlpha,minHoodAngle,maxHoodAngle));
        vel = FromTicksToRPM(velocity);
    }
    private static double FromRadsToPos(double angle) {
        double pos = ((minHoodPos - maxHoodPos) / (minHoodAngle - maxHoodAngle)) * ( angle - minHoodAngle) + minHoodPos;
        return Math.clamp(pos,minHoodPos,maxHoodPos);
    }
    private static double FromTicksToRPM(double velocity) {
        return Math.clamp(velocity * 60 / 28,1300,2200);
    }
}