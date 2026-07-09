package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;

public class BetterServo {
    public enum State{
        PROFILE,
        NORMAL,
    };

    public State state;
    public double targetPos;
    public ServoImplEx servo;
    public double maxVel = 0, acc = 0, dec=0;
    public BetterMotionProfile profile = new BetterMotionProfile(maxVel,acc,dec);
    public BetterServo(ServoImplEx servo,State state,double initPos){
        targetPos = initPos;
        this.state = state;
        servo.setPosition(targetPos);
    };
    public BetterServo(ServoImplEx servo,State state,double initPos,boolean reversed) {
        targetPos = initPos;
        this.state = state;
        if (reversed) {
            servo.setDirection(Servo.Direction.REVERSE);
        }
        servo.setPosition(targetPos);
    }
    public void setProfileCoefficients(double maxVel,double acc, double dec){
        profile.maxVelocity = maxVel;
        profile.acceleration = acc;
        profile.deceleration = dec;
    }
    public void setTargetPos(double position){
        targetPos = position;
    }
    public void setPWMRange(double usPulseLower, double usPulseUpper){
        servo.setPwmRange(new PwmControl.PwmRange(usPulseLower,usPulseUpper));
    }
    public void update(){
        if (state == State.PROFILE){
            profile.update();
            profile.setMotion(profile.getPosition(),targetPos, profile.getVelocity());
            servo.setPosition(profile.getPosition());

        }
        else if (state == State.NORMAL){
            servo.setPosition(targetPos);
        }
    }
}
