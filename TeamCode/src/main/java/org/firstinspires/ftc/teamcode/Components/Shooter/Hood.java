package org.firstinspires.ftc.teamcode.Components.Shooter;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Config
public class Hood {
    ServoImplEx servo;
    public static double k = 0.00045;
    public static double idlePos = 0,shootPos = 0;
    public enum State{
        IDLE(idlePos),
        SHOOT(shootPos);
        double position;
        State(double position){
            this.position = position;
        }
    }
    public State state = State.IDLE;
    public Hood(){
        servo = Hardware.ssh5;
        servo.setDirection(Servo.Direction.FORWARD);
    }
    public void update(){
        updateState();
        updatePosition();
        updateHardware();
    }
    private void updateHardware(){
        servo.setPosition(state.position);
    }
    private void updateState(){
        switch (state){
            case IDLE:
                idlePos = calculateHoodPos(Odo.delta);
            case SHOOT:
                shootPos = calculateHoodPos(Odo.delta) + k*(FlyWheel.targetVelocity-FlyWheel.currentVelocity);
                break;
        }
    }
    private void updatePosition(){
        State.IDLE.position = idlePos;
        State.SHOOT.position = shootPos;
    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){
        return state;
    }
    public double getPosition(){
        return state.position;
    }
    public double calculateHoodPos(double distance){
        return Math.clamp((-1)*(3.31142*Math.pow(10,-8))*Math.pow(distance,2)+0.000287928*distance-0.237023,0.01,0.495);
    }

}
