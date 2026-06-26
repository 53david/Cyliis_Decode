package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.transfer;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;

@Configurable
public class Latch {

    public static double transPos = 0.385;
    public static double idlePos = 0.16;
    public double currentPos;

        public enum State{
        IDLE,
        TRANSFER,
    };
    public static State state;
    public static double maxVel=20, acc=16, dec=16;
    BetterMotionProfile profile;
    public Latch(){
        transfer.setPosition(idlePos);
        state = State.IDLE;
        currentPos = idlePos;
        profile = new BetterMotionProfile(maxVel,acc,dec);
        profile.setMotion(currentPos, currentPos, 0);


    }
    public void update(){
        stateUpdate();
        profile.update();
        if(profile.finalPosition != currentPos)
            profile.setMotion(profile.getPosition(), currentPos, profile.getVelocity());
        transfer.setPosition(profile.getPosition());
    }
    public void stateUpdate(){
        switch (state){
            case IDLE:
                currentPos = idlePos;
                break;
            case TRANSFER:
                currentPos = transPos;
                break;
        }
    }
    public void test(){
        stateUpdate();
        profile.update();
        if(profile.finalPosition != currentPos)
            profile.setMotion(profile.getPosition(), currentPos, profile.getVelocity());
        transfer.setPosition(profile.getPosition());
        telemetryM.addData("pos",profile.getPosition());
        telemetryM.addData("vel",profile.getVelocity());
        telemetryM.update();
    }
}