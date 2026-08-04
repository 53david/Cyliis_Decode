package org.firstinspires.ftc.teamcode.Components.Intake;

import android.security.identity.EphemeralPublicKeyNotFoundException;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@Config
public class Storage {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime failSafe = new ElapsedTime();
    public static double angle;
    public double position = 0;
    public static double specialPos = Math.toRadians(250);
    public static double ballPos1 = Math.toRadians(70),ballPos3 = Math.toRadians(190),ballPos2 = Math.toRadians(310);
    public static double Kp = 0.57,KP = 0.9;
    public static double Kd = 0.02,KD = 0.023;
    public static double Ks = 0;
    PIDController pid = new PIDController(Kp,0,Kd);
    public static double error = 0;
    AnalogInput encoder;
    CRServo motor;

    public enum State{
        BALL1(ballPos1), BALL2(ballPos2), BALL3(ballPos3),
        GOINGBALL1(BALL1,ballPos1), GOINGBALL2(BALL2,ballPos2), GOINGBALL3(BALL3,ballPos3),
        TRANSFER(specialPos), GOINGTRANSFER(TRANSFER,specialPos),
        SHOOT(GOINGBALL1);
        State nextState;
        double position;
        State(State nextState){
            this.nextState = nextState;
            this.position = 1e9;
        }
        State(State nextState,double position){
            this.position = position;
            this.nextState = nextState;
        }
        State(double position){
            this.position = position;
            this.nextState=this;
        }

    }
    public State state = State.GOINGBALL1;

    public Storage(){

        motor= Hardware.sch3;
        encoder = Hardware.analogInput;
        State.BALL1.nextState = State.GOINGBALL2;
        State.BALL2.nextState = State.GOINGBALL3;
        State.BALL3.nextState = State.GOINGTRANSFER;

        timer.startTime();
        failSafe.startTime();
    }

    public void update(){
        updatePositions();
        updateState();
        updateAngle();
        updatePID();
        updateHardware();
    }
    private void updateHardware(){

        if (state != State.SHOOT){
            double power = pid.calculate(0,-error) + Ks *Math.signum(error);

            motor.setPower(power);
        }

        if(state==State.SHOOT)
        {
            motor.setPower(-1);
        }

    }
    private void updateState(){
        switch (state){
            case GOINGBALL1:
            case GOINGBALL2:
            case GOINGBALL3:
                if (Math.abs(error)<0.32)state=state.nextState;
                break;
            case GOINGTRANSFER:
                if (Math.abs(error)<0.2)state=state.nextState;
                break;
            case BALL1:
            case BALL2:
            case BALL3:
                break;
            case TRANSFER:
                timer.reset();
                break;
            case SHOOT:
                if (timer.seconds()>0.315)state=state.nextState;
                break;

        }
    }
    private void updatePID(){
        if (state == State.SHOOT){
            pid.kp = 0;
            pid.kd = 0;
        }
        else {
            if (Math.abs(error) > 0.24) {
                pid.kp = Kp;
                pid.kd = Kd;
            } else {
                pid.kp = KP;
                pid.kd = KD;
            }
        }
    }
    private void updateAngle(){
        angle = Math.abs(encoder.getVoltage()/encoder.getMaxVoltage()) * Math.PI*2;
        error = position - angle;
        if (Math.abs(error)>Math.PI){
            error = -Math.signum (error) * (2 * Math.PI - Math.abs(error));
        }
    }
    private void updatePositions(){

        State.BALL1.position = ballPos1;
        State.BALL2.position = ballPos2;
        State.BALL3.position = ballPos3;
        State.GOINGBALL1.position = ballPos1;
        State.GOINGBALL2.position = ballPos2;
        State.GOINGBALL3.position = ballPos3;

    }
    public void shoot(){
        if (state == State.TRANSFER) state = State.SHOOT;
    }
    public State getState(){
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
    public void goNext()
    {
        if (state == State.BALL1 || state == State.BALL2 || state == State.BALL3) state=state.nextState;
    }
    public void byPass(){
        if (state != State.TRANSFER && state!=State.GOINGTRANSFER) this.state = State.GOINGTRANSFER;
    }

}
