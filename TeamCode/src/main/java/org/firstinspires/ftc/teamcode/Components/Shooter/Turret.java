package org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.servo1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.servo2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Turret {
    public static double goalPositionX = 0, goalPositionY = 650;
    public static double targetAngle = 0;
    public double targetPosition = 0.5;
    public double targetPosition1,targetPosition2;
    public double maxAngle = Math.PI * 2;
    public static double offset = 0;
    public static double dx = 0;
    public static double dy = 0;


    public enum AllianceState {
        RED,
        BLUE,
    }

    public static AllianceState allianceState;

    public Turret() {
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);

    }

    private void updateServosPosition() {
        double rAngle = targetAngle;
        rAngle -= Odo.getHeading();
        rAngle = normalizeRadians(rAngle);
        rAngle = rAngle / maxAngle;
        targetPosition = rAngle;
        targetPosition1 = Math.max(0.005, targetPosition);
        targetPosition1 = Math.min(9.995, targetPosition);
        targetPosition2 = Math.max(0.005, targetPosition);
        targetPosition2 = Math.min(9.995, targetPosition);
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);

    }

    public void updateAngle() {
        dx = goalPositionX - Odo.getX();
        dy = goalPositionY - Odo.getY();
        targetAngle = Math.atan2(dy, dx);
        targetAngle += offset;

    }

    public void update() {
        stateUpdate();
        updateAngle();
        updateServosPosition();
        if (gm1.ps && prevgm1.ps!=gm1.ps){
            offset = 0;
        }
    }

    public void stateUpdate() {
        switch (allianceState) {
            case BLUE:
                goalPositionX = 0;
                goalPositionY = 575;
                break;
            case RED:

                break;
        }

    }

    public double normalizeRadians(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;

    }
}