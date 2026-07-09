package org.firstinspires.ftc.teamcode.Math;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class ShooterCalculator {

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0.15;
    public static double Kv = 0.00044;
    public static double Ka = 0.00635;
    public static double regression = 0.00045;
    public static double g = 9810.0;
    public static double goalHeight = 735;
    public static double theta = -Math.toRadians(30);
    public static double targetAngle = 0;
    public static double hoodPos = 0.5;
    public static double vel = 1200;
    public static double gv = 0;

    public void updateTrajectory(double robotX, double robotY, double velX, double velY, double goalX, double goalY) {
        double distance = Math.hypot(goalX - robotX, goalY - robotY);
        double gAngle = Math.atan2(goalY - robotY, goalX - robotX);

        double alpha = Math.atan((2 * goalHeight / distance) - Math.tan(theta));
        double v0 = Math.sqrt(Math.abs((g * Math.pow(distance, 2)) / (2 * Math.pow(Math.cos(alpha), 2) * (distance * Math.tan(alpha) - goalHeight))));

        double rAngle = Math.atan2(velY, velX) - gAngle;
        double vParallel = -Math.cos(rAngle) * Odo.avgVel();
        double vPerpendicular = Math.sin(rAngle) * Odo.avgVel();

        double time = distance / (v0 * Math.cos(alpha));

        double vCompensation = (distance / time) + vParallel;
        double vx = Math.hypot(vCompensation, vPerpendicular);
        double vy = v0 * Math.sin(alpha);

        double newAlpha = Math.atan2(vy, vx);
        double velocity = Math.hypot(vx,vy);
        hoodPos = FromRadsToPos(newAlpha);
        vel = FromVelocityToTicks(velocity);

        double turretOffset = Math.atan2(vPerpendicular, vCompensation);
        targetAngle = gAngle -turretOffset;
    }
    public static double fwVel(double delta) {
        return Math.clamp(0.318182*delta+1081.81818 - 25,1300,2300);
    }
    public static double hoodAngle(double delta){
        return Math.clamp((-0.0000631481*Math.pow(delta,2)
                +0.413426*delta-180.32593) * 0.001,0.11,0.55);
    }
    private static double FromRadsToPos(double angle) {
        return Math.clamp(-0.742308*angle+0.882,0.11,0.55);
    }

    private static double FromVelocityToTicks(double velocity) {
        return Math.clamp(velocity / Math.PI, 1300, 2200);

    }
}
