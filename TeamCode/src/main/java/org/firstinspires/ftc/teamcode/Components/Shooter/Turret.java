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
    public static double goalPositionX = 0, goalPositionY = 0;
    public static double targetAngle = 0;
    public double targetPosition = 0.5;
    public double maxAngle = Math.PI*2;
    public static double dx =0;
    public static double dy = 0;
    public static double a = 1.1;
    public double t = 0;
    public static double angleOffset = 0;
    public enum State{
        IDLE,
        ACTIVE,
    }
    public enum AllianceState {
        RED,
        BLUE,
    }
    public static AllianceState allianceState;
    public static State state;

    public Turret() {
        state = State.ACTIVE;
        servo1.setPwmRange(new PwmControl.PwmRange(500 , 2500));
        servo2.setPwmRange(new PwmControl.PwmRange(500 , 2500));

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.REVERSE);

    }

    private void updateServosPosition() {
        double rAngle = -Odo.getHeading();
        if (Odo.avgVel()>750){
            rAngle += ShooterCalculator.targetAngle;
        }
        else {
            rAngle += targetAngle;
        }
        if (LimeLight.streamState == LimeLight.StreamState.STREAM && LimeLight.getPipeline() == 2){
            rAngle += Math.toRadians(LimeLight.getHeading());
        }
        rAngle = normalizeRadians(rAngle);
        rAngle = rAngle / maxAngle;
        targetPosition = rAngle;
        targetPosition = Math.max(0.007, targetPosition);
        targetPosition = Math.min(1 - 0.007, targetPosition);
        servo1.setPosition(targetPosition);
        servo2.setPosition(targetPosition);

    }
    public void updateAngle() {
        dx = goalPositionX - Odo.getRawX();
        dy = goalPositionY - Odo.getRawY();
        targetAngle = Math.atan2(dy,dx);

    }

    public void update() {
        stateUpdate();
        updateAngle();
        if (gm1.dpad_down && prevgm1.dpad_down){
            state = State.IDLE;
        }
    }
    public void stateUpdate(){
        switch (allianceState){
            case BLUE:
                if(gm1.right_stick_y>0.5 && prevgm1.right_stick_y<0.5)Odo.offsetY+=40;
                if(gm1.right_stick_y<-0.5 && prevgm1.right_stick_y>-0.5)Odo.offsetY-=40;
                if(gm1.right_stick_x>0.5 && prevgm1.right_stick_x<0.5)Odo.offsetX-=40;
                if(gm1.right_stick_x<-0.5 && gm1.right_stick_x>-0.5)Odo.offsetX+=40;
                goalPositionX = 20; goalPositionY = 820;
                break;
            case RED:
                goalPositionX = -20; goalPositionY = -820;
                if(gm1.right_stick_y>0.5 && prevgm1.right_stick_y<0.5)Odo.offsetY-=40;
                if(gm1.right_stick_y<-0.5 && prevgm1.right_stick_y>-0.5)Odo.offsetY+=40;
                if(gm1.right_stick_x>0.5 && prevgm1.right_stick_x<0.5)Odo.offsetX+=40;
                if(gm1.right_stick_x<-0.5 && gm1.right_stick_x>-0.5)Odo.offsetX-=40;
                break;
        }
        switch (state){
            case IDLE:
                servo1.setPosition(0.05);
                servo2.setPosition(0.05);
                break;
            case ACTIVE:
                updateServosPosition();
                break;
        }
    }
    public double normalizeRadians(double angle){
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += (2.0 * Math.PI);
        return angle;

    }
    public void test(){

        telemetryM.addData("Angle",targetAngle);
        telemetryM.addData("X",Odo.getX());
        telemetryM.addData("y",Odo.getY());
        telemetryM.addData("Heading",Odo.getHeading());
        telemetryM.addData("Goal X",goalPositionX);
        telemetryM.addData("Goal Y",goalPositionY);
        telemetryM.update();
        stateUpdate();
        updateAngle();
        updateServosPosition();

    }
}