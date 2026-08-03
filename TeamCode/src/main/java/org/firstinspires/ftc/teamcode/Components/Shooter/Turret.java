package org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.servo1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.servo2;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Turret {
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

    public void updateAngle() {
        dx = goalPositionX - (Odo.predictedX + tx*Math.cos(Odo.heading) + Odo.xRobotVelocity * a);
        dy = goalPositionY - (Odo.predictedY + tx*Math.sin(Odo.heading) + Odo.yRobotVelocity * a);
        targetAngle = Math.atan2(dy, dx);

    }

    public void update() {
        updateAngle();
        updateServosPosition();

    }
    public double normalizeRadians(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;

    }
}