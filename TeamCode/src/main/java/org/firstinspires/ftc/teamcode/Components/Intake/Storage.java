package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.encoder;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.proximitySensor;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.spin;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Math.PIDController;


public class Storage {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime failSafe = new ElapsedTime();
    public static double angle;
    public static boolean isTransferReady = false;
    public double position = 0;
    public static double specialPos = Math.toRadians(250);
    public static double ballPos1 = Math.toRadians(70),ballPos3 = Math.toRadians(190),ballPos2 = Math.toRadians(310);
    public static double Kp = 0.9;
    public static double KP = 0.57;
    public static double Kd = 0.023;
    public static double KD = 0.02;
    public static double Ks = 0;
    PIDController pid = new PIDController(Kp,0,Kd);
    public static double error = 0;

    public enum State{
        RESET,
        BALL1(ballPos1),
        BALL2(ballPos2),
        BALL3(ballPos3),
        GOINGBALL1(BALL1,ballPos1),
        GOINGBALL2(BALL2,ballPos2),
        GOINGBALL3(BALL3,ballPos3),
        TRANSFER(specialPos),
        GOINGTRANSFER(TRANSFER,specialPos),
        SHOOT(RESET);
        State nextState;
        double position;
        State(){

        }
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
        }
    }
    public static State state;
    public Storage(){
        timer.startTime();
        state = State.RESET;
        failSafe.startTime();
        isTransferReady = false;
    }
    public void stateUpdate(){
        State.BALL1.nextState = State.GOINGBALL2;
        State.BALL2.nextState = State.GOINGBALL3;
        State.BALL1.nextState = State.TRANSFER;
        switch (state){
            case GOINGBALL1:
            case GOINGBALL2:
            case GOINGBALL3:
                break;
            case GOINGTRANSFER:
                if (Math.abs(error)<0.32)state = state.nextState;
                break;
            case BALL1:
            case BALL2:
            case BALL3:
                break;
            case TRANSFER:
                Latch.state = Latch.State.TRANSFER;
                timer.reset();
                break;
            case SHOOT:
                spin.setPower(-1);
                if (timer.seconds()>0.35)state = state.nextState;
            case RESET:
                position = specialPos;
                state.nextState = State.GOINGBALL1;
                break;

        }
    }
    public void update(){
        stateUpdate();
        updateAngle();
        spinUpdate();
        updatePID();
        if (!isMoving()){
            if (failSafe.seconds()>1 && failSafe.seconds()<2){
                spin.setPower(0);
                return;
            }
            if (failSafe.seconds()>2){
                failSafe.reset();
            }
        }
        if ((state == State.GOINGBALL1 || state == State.GOINGBALL2 || state == State.GOINGBALL3 || state == State.GOINGTRANSFER) && !isMoving()) {
            state = state.nextState;
        }
        if ((state == State.BALL1 || state == State.BALL2 || state == State.BALL3) && isBallInStorage()) {
            state = state.nextState;
        }
    }
    public static void shoot(){
        if (Storage.state == State.TRANSFER) state = State.SHOOT;
    }
    public void spinUpdate(){
        double power = pid.calculate(0,-error) + Ks *Math.signum(error);
        if (state != State.SHOOT){
            spin.setPower(power);
        }

    }
    public void updatePID(){
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
    public void updateAngle(){
        angle = Math.abs(encoder.getVoltage()/encoder.getMaxVoltage()) * Math.PI*2;
        error = position - angle;
        if (Math.abs(error)>Math.PI){
            error = -Math.signum (error) * ( 2 * Math.PI - Math.abs(error));
        }
    }
    public static boolean isBallInStorage(){
        return !proximitySensor.getState();
    }
    public static boolean isMoving(){
        return (Storage.state != State.TRANSFER && Math.abs(error) > 0.24) || (Math.abs(error)> 0.12 && Storage.state == State.TRANSFER);
    }

}