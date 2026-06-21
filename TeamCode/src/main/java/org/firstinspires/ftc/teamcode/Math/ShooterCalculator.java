package org.firstinspires.ftc.teamcode.Math;

import com.bylazar.configurables.annotations.Configurable;
@Configurable
public class ShooterCalculator {

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0.15;
    public static double Kv = 0.00044;
    public static double Ka = 0.00635;
    public static double regression = 0.005;

    public static double fwVel(double delta) {
        return Math.clamp(0.37415 * delta+1155.78231,1500,2400);
    }
    public static double hoodAngle(double delta){
        return Math.clamp((-0.0000631481*Math.pow(delta,2)
                +0.413426*delta-180.32593) * 0.001,0.11,0.496);
    }
}