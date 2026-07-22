package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.encoder;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.proximitySensor;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.spin;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Storage {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime failSafe = new ElapsedTime();
    public static double angle;
    public static boolean isTransferReady = false;
    public static double target = Math.PI/4.0;
    public static double nrBalls =  0;
    public static double specialPos = Math.toRadians(250);
    public static double ballPos1 = Math.toRadians(70),ballPos3 = Math.toRadians(190),ballPos2 = Math.toRadians(310);
    public static double Kp = 0.57;
    public static double KP = 0.57;
    public static double Kd = 0.013;
    public static double KD = 0.013;
    public static double Ks = 0;
    PIDController pid = new PIDController(Kp,0,Kd);
    public static double error = 0;

    public enum State{
        BALL1,
        BALL2,
        BALL3,
        TRANSFER,
        SHOOT,
        RESET,
    }
    public static State state,nextState;
    public Storage(){
        timer.startTime();
        state = State.RESET;
        failSafe.startTime();
        isTransferReady = false;
    }
    public void stateUpdate(){
        error = target - angle;
        switch (state){
            case BALL1:
                nrBalls = 1;
                target = ballPos1;
                nextState = State.BALL2;
                break;

            case BALL2:
                target = ballPos2;
                nrBalls = 1;
                nextState = State.BALL3;
                break;

            case BALL3:
                target = ballPos3;
                nrBalls = 2;
                nextState = State.TRANSFER;
                break;

            case TRANSFER:
                target = specialPos;
                nextState = State.SHOOT;
                if(!IsStorageSpinning()){
                    isTransferReady = false;
                    Latch.state = Latch.State.TRANSFER;
                }
                else {
                    isTransferReady = true;
                }
                if (isAutonomousActive){
                    timer.reset();
                }
                break;

            case SHOOT:
                Hood.state = Hood.State.SHOOT;
                spin.setPower(Odo.power);
                nextState = State.RESET;
                if (timer.seconds()>0.5){
                    goNext();
                }
                break;

            case RESET:
                target = ballPos1; nrBalls = 0;
                Latch.state = Latch.State.IDLE;
                Hood.state = Hood.State.IDLE;
                nextState = State.BALL1;
                break;

        }
    }
    public void goNext(){
        state = nextState;
    }
    public void update(){
        updatePID();
        stateUpdate();
        updateAngle();
        spinUpdate();
        if (IsStorageSpinning() && !isAutonomousActive){
            if (failSafe.seconds()>1 && failSafe.seconds()<2){
                spin.setPower(0);
                return;
            }
            if (failSafe.seconds()>2){
                failSafe.reset();
            }
        }
        else if (!IsStorageSpinning() && !isAutonomousActive){
            failSafe.reset();
        }
        if (gm1.circle && prevgm1.circle!= gm1.circle && nrBalls>=1){
            state = State.TRANSFER;
        }
        if ((state == State.BALL1 || state == State.BALL2 || state ==State.BALL3) && !IsStorageSpinning() && Storage.isBallInStorage()){
            goNext();
        }
        if (state == State.TRANSFER && gm1.cross && prevgm1.cross != gm1.cross){
            timer.reset();
            goNext();
        }
        if (state == State.RESET && !IsStorageSpinning()){
            goNext();
        }
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
        error = target - angle;
        if (Math.abs(error)>Math.PI){
            error = -Math.signum (error) * ( 2 * Math.PI - Math.abs(error));
        }
    }
    public static boolean isBallInStorage(){
        return !proximitySensor.getState();
    }
    public static boolean IsStorageSpinning(){
        return Math.abs(error) > 0.24;
    }

}