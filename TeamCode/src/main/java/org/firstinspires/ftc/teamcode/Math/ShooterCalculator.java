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
    public static double Ka = 0.00545;
    public static double regression = 0.0004;
    public static double targetAngle = 0;
    public static double hoodPos = 0.5;
    public static double vel = 1200;
    public static double time = 0.0425;

    public void updateTrajectory(double robotX, double robotY, double velX, double velY, double goalX, double goalY) {
        double distance = Math.hypot(goalX - robotX, goalY - robotY);
        double gAngle = Math.atan2(goalY - robotY, goalX - robotX);

        double currentVel = Odo.avgVel();
        if (currentVel < 100) {
            currentVel = 0;
        }

        double rAngle = Math.atan2(velY, velX) - gAngle;
        double vParallel = -Math.cos(rAngle) * currentVel;
        double vPerpendicular = Math.sin(rAngle) * currentVel;
        double vDistance = (distance + (vParallel * time));

        hoodPos = hoodAngle(vDistance);
        vel = fwVel(vDistance);
        hoodPos = hoodAngle(vDistance);
        double vCompensation = (distance / time) + vParallel;
        double turretOffset = Math.atan2(vPerpendicular, vCompensation);

        targetAngle = gAngle - turretOffset*1.1;
    }
    public static double fwVel(double delta) {
        return Math.clamp(0.360465*delta+992.51163,1300,2100);
    }
    public static double hoodAngle(double delta){
        return Math.clamp(((7.62026*Math.pow(10,-8))*Math.pow(delta,3)-0.000520261*Math.pow(delta,2)+1.2542*delta-686.98119)*0.001,0.11,0.496);
    }

}
