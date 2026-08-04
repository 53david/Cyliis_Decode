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
    public static double redX = 0,redY = -840;

    public static double blueX = 0,blueY = 840;
    public double targetAngle = 0;
    public static double a = 0.05;
    public double targetPosition = 0.5;
    public static boolean pause = false;
    public double maxAngle = Math.PI * 2;
    public static double offset = 0;
    public static double dx = 0, tx =17.995;
    public static double dy = 0, ty = 0;
    public enum State{
        RED(redX,redY),
        BLUE(blueX,blueY),
        IDLE;
        double x,y;
        State(){

        }
        State(double x,double y){
            this.x = x;
            this.y = y;
        }
    }
    public State state;
    public Turret() {
        servo1 = Hardware.ssh1;
        servo2 = Hardware.ssh2;
        servo1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);

    }
    public void update() {
        updateTargetPos();
        updateAngle();
        updateServosPosition();
        updateState();

    }

    private void updateServosPosition() {
        if (pause){
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
            return;
        }
        double rAngle = targetAngle + offset;
        rAngle -= Odo.heading;
        rAngle = normalizeRadians(rAngle);
        rAngle = rAngle / maxAngle;
        targetPosition = rAngle;
        targetPosition = Math.max(0.09  , targetPosition);
        targetPosition = Math.min(0.95, targetPosition);
        servo1.setPosition(targetPosition);
        servo2.setPosition(targetPosition);

    }

    private void updateAngle() {
        dx = state.x - (Odo.predictedX + tx*Math.cos(Odo.heading) + Odo.xRobotVelocity * a);
        dy = state.y - (Odo.predictedY + tx*Math.sin(Odo.heading) + Odo.yRobotVelocity * a);
        targetAngle = Math.atan2(dy, dx);

    }
    public void updateState(){
        switch (state){
            case IDLE:
                pause = true;
                break;
            case BLUE:
            case RED:
                pause = false;
                break;
        }
    }
    public void updateTargetPos(){
        State.RED.x = redX; State.RED.y = redY;
        State.BLUE.x = blueX; State.BLUE.y = blueY;

        Odo.goalPositionX = state.x;
        Odo.goalPositionY = state.y;
    }
    private double normalizeRadians(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;

    }
    public void setState(State state){
        this.state = state;
    }
}