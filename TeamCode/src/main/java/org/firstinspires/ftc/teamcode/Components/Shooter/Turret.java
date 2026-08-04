package org.firstinspires.ftc.teamcode.Components.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Config
public class Turret {
    ServoImplEx servo1,servo2;
    public static double goalPositionX = 0, goalPositionY = 840;
    public static double targetAngle = 0;
    public static double a = 0.05;
    public double targetPosition = 0.5;
    public static boolean pause = false;
    public double maxAngle = Math.PI * 2;
    public static double offset = 0;
    public static double dx = 0, tx =18.34;
    public static double dy = 0, ty = 0;

    public Turret() {
        servo1 = Hardware.ssh1;
        servo2 = Hardware.ssh2;
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);

    }

    private void updateServosPosition() {
        double rAngle = targetAngle + offset;
        rAngle -= Odo.heading;
        rAngle = normalizeRadians(rAngle);
        rAngle = rAngle / maxAngle;
        targetPosition = rAngle;
        targetPosition = Math.max(0.09  , targetPosition);
        targetPosition = Math.min(0.95, targetPosition);
        if (pause){
            targetPosition = 0.5;
        }
        servo1.setPosition(targetPosition);
        servo2.setPosition(targetPosition);

    }

    private void updateAngle() {
        dx = goalPositionX - (Odo.predictedX + tx*Math.cos(Odo.heading) + Odo.xRobotVelocity * a);
        dy = goalPositionY - (Odo.predictedY + tx*Math.sin(Odo.heading) + Odo.yRobotVelocity * a);
        targetAngle = Math.atan2(dy, dx);

    }

    public void update() {
        updateAngle();
        updateServosPosition();

    }
    private double normalizeRadians(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;

    }
}