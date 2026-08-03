package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;

@Config
public class Latch {
    public double error = 0,profilePos = 0;
    public static double transPos = 0.39;
    public static double idlePos = 0.16;
    public double target;

        public enum State{
            TRANSFER(transPos),
            GOINGTRANSFER(transPos,TRANSFER),
            IDLE(idlePos,GOINGTRANSFER),
            GOINGIDLE(idlePos,IDLE);
        double position;
        State nextState;
        State(double position){
            this.position = position;
        }
        State(double position,State nextState){this.position= position;this.nextState = nextState;
        }

        };
    public static State state;
    public static double maxVel=20, acc=16, dec=16;
    BetterMotionProfile profile;
    public Latch(){
        profile = new BetterMotionProfile(maxVel,acc,dec);
        profile.setMotion(target, target, 0);
    }

    public void update(){

        updatePositions();
        stateUpdate();
        profile.update();
        target = state.position;
        profilePos = profile.getPosition();
        error = Math.abs(target-profilePos);
        updateHardware();

    }
    public void updateHardware(){
        if(profile.finalPosition != target)
            profile.setMotion(profilePos, target, profile.getVelocity());
        transfer.setPosition(profilePos);
    }
    public void updatePositions(){
        State.TRANSFER.nextState = State.GOINGIDLE;
        State.GOINGTRANSFER.position = transPos;
        State.TRANSFER.position = transPos;
        State.IDLE.position = idlePos;
        State.GOINGIDLE.position = idlePos;
    }
    public void stateUpdate(){
        switch (state){
            case IDLE:
            case TRANSFER:
                break;
            case GOINGTRANSFER:
            case GOINGIDLE:
                if (error<0.05){
                    state = state.nextState;
            }
                break;
        }
    }
}