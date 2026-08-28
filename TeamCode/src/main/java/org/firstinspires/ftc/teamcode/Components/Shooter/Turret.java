package org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.pp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Config
public class Turret {
    ServoImplEx servo1,servo2;
    double x = 0;
    public static double redX = 0,redY = -840;
    public static double blueX = 0,blueY = 840;
    public double targetAngle = 0;
    public static double a = 0.375;
    public double targetPosition = 0.5;
    public boolean pause = false;
    public double maxAngle = Math.PI * 2;
    public static double offset = 0;
    public static double dx = 0, tx =17.793;
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
    public State state = State.BLUE;
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
            servo1.setPosition(0.5 + x);
            servo2.setPosition(0.5 + x);
            return;
        }
        double rAngle = targetAngle + offset;
        rAngle -= Odo.heading;
        rAngle = normalizeRadians(rAngle);
        rAngle = rAngle / maxAngle;

        targetPosition = rAngle;
        targetPosition = Math.max(0.05 , targetPosition);
        targetPosition = Math.min(0.9, targetPosition);

        servo1.setPosition(targetPosition);
        servo2.setPosition(targetPosition);

    }

    private void updateAngle() {

        dx = state.x - (pp.getPosX(DistanceUnit.MM)+Odo.offsetX + pp.getVelX(DistanceUnit.MM) * a + tx*Math.cos(Odo.getHeading()));
        dy = state.y - (pp.getPosY(DistanceUnit.MM)+Odo.offsetY + pp.getVelY(DistanceUnit.MM) * a +tx*Math.sin(Odo.getHeading()));
        targetAngle = Math.atan2(dy, dx);

    }
    public void updateState(){
        switch (state){
            case IDLE:
                pause = true;
                break;
            case BLUE:
                x = -0.02;
                pause = false;
                break;
            case RED:
                x = 0.01;
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