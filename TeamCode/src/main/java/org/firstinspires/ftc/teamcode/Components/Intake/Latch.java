package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.transfer;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;

@Configurable
public class Latch {

    public static double transPos = 0.39;
    public static double idlePos = 0.16;
    public double target;

        public enum State{
        IDLE(idlePos),
        TRANSFER(transPos);
        double position;
        State(){

        }
        State(double position){
            this.position = position;
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
        stateUpdate();
        profile.update();
        target = state.position;
        if(profile.finalPosition != target)
            profile.setMotion(profile.getPosition(), target, profile.getVelocity());
        transfer.setPosition(profile.getPosition());
    }
    public void stateUpdate(){
        switch (state){
            case IDLE:
            case TRANSFER:
                break;
        }
    }
}