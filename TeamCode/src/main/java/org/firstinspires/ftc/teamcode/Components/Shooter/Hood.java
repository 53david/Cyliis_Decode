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
    int i = 0;
    public double[] v={
            0.11,
            0.15,
            0.22,
            0.24,
            0.25,
            0.27,
            0.33,
            0.34,
            0.36,
            0.42,
    };
    public enum State{
        IDLE(idlePos),
        SHOOT(shootPos);
        double position;
        State(double position){
            this.position = position;
        }
    }
    public static State state = State.IDLE;
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
                i = Math.max((Odo.delta/100-8),0);
                i = Math.min(i,v.length-1);
                idlePos = v[i];
            case SHOOT:
                i = Math.max((Odo.delta/100-8),0);
                i = Math.min(i,v.length-1);
                shootPos = v[i] + k*(FlyWheel.targetVelocity-FlyWheel.currentVelocity);
                break;
        }
    }
    private void updatePosition(){
        State.IDLE.position = idlePos;
        State.SHOOT.position = shootPos;
    }

}
