package org.firstinspires.ftc.teamcode.Components.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@Config
public class Latch {
    public double profilePos = 0;
    public static double transferPos = 0.3975;
    public static double idlePos = 0.16;
        public enum State{
            TRANSFER(transferPos),
            GOINGTRANSFER(transferPos,TRANSFER),
            IDLE(idlePos),
            GOINGIDLE(idlePos,IDLE);
        double position;
        State nextState;
        State(double position){
            this.position = position;
            this.nextState=this;
        }
        State(double position,State nextState){
            this.position= position;
            this.nextState = nextState;
        }

        };
    private State state;
    public static double maxVel=20, acc=16, dec=16;
    BetterMotionProfile profile;
    ServoImplEx servo;
    public Latch(){
        servo= Hardware.ssh0;
        state=State.IDLE;
        profile = new BetterMotionProfile(maxVel,acc,dec);
        profile.setMotion(state.position, state.position, 0);
    }

    public void update(){

        updatePositions();
        updateState();
        updateHardware();

        profile.update();
    }
    private void updateHardware(){

        if(profile.finalPosition != state.position)
            profile.setMotion(profilePos, state.position, profile.getVelocity());

        servo.setPosition(profile.getPosition());
    }
    private void updatePositions(){
        State.GOINGTRANSFER.position = transferPos;
        State.TRANSFER.position = transferPos;
        State.IDLE.position = idlePos;
        State.GOINGIDLE.position = idlePos;
    }
    private void updateState(){
        switch (state){
            case IDLE:
            case TRANSFER:
                break;
            case GOINGTRANSFER:
            case GOINGIDLE:
                if(profile.getPosition()==profile.finalPosition)
                    state = state.nextState;
                break;
        }
    }
    public void setState(State state)
    {
        if(state.nextState!=this.state)
            this.state=state;
    }

    public State getState()
    {
        return state;
    }
}