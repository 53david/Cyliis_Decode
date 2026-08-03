package org.firstinspires.ftc.teamcode.Components.Shooter;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.hood;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Hood {
    public static double k = 0.00045;
    public static double pos = 0;
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
        IDLE(pos),
        SHOOT(pos);
        double position;
        State(){

        }
        State(double position){
            this.position = position;
        }
    }
    public static State state = State.IDLE;
    public void updateState(){
        switch (state){
            case IDLE:
                i = Math.max((Odo.delta%100-8),0);
                i = Math.min(i,v.length-1);
                pos = v[i];
            case SHOOT:
                i = Math.max((Odo.delta%100-8),0);
                i = Math.min(i,v.length-1);
                pos = v[i] + k*(FlyWheel.targetVelocity-FlyWheel.currentVelocity);
                break;
        }
    }
    public Hood(){
        hood.setDirection(Servo.Direction.FORWARD);
    }
    public void update(){
        updateState();
        hood.setPosition(state.position);
    }
    public void tune(){
        hood.setPosition(pos);
    }
}
