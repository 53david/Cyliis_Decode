package org.firstinspires.ftc.teamcode.Components.Intake;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.encoder;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.spin;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Storage {
    ElapsedTime timer = new ElapsedTime();
    public static boolean isTransferReady = false;
    public static double target = Math.PI/4.0;
    public static double nrBalls =  0;
    public static double resetPos = Math.toRadians(120);
    public static double specialPos = Math.toRadians(250);
    public static double ballPos1 = Math.toRadians(70),ballPos2 = Math.toRadians(190),ballPos3 = Math.toRadians(310);
    public static double Kp = 0.6;
    public static double Kd = 0.02;
    public static double Ks = 0;
    PIDController pid = new PIDController(Kp,0,Kd);
    public enum State{
        BALL1,
        BALL2,
        BALL3,
        TRANSFER,
        SHOOT,
        RESET,
    }
    public static State state;
    public Storage(){
        timer.startTime();
        state = State.RESET;
        isTransferReady = false;
    }
    public void stateUpdate(){

        switch (state){
            case BALL1:

                target = ballPos1;

                if (!IsStorageSpinning() && ColorDetection.isBallInStorage()){
                    state = State.BALL2;
                    nrBalls = 1;
                }
                break;

            case BALL2:

                target = ballPos2;

                if (!IsStorageSpinning() && ColorDetection.isBallInStorage()){
                    state = State.BALL3;
                    nrBalls = 2;
                }
                break;

            case BALL3:

                target = ballPos3;

                if (!IsStorageSpinning() && ColorDetection.isBallInStorage()){
                    state = State.TRANSFER;
                    nrBalls = 3;
                    isTransferReady = true;
                }
                break;

            case TRANSFER:
                target = specialPos;
                isTransferReady = false;
                if(!IsStorageSpinning() && timer.seconds()>0.25){
                    Latch.state = Latch.State.TRANSFER;
                }
                if (!IsStorageSpinning() && gm1.cross && prevgm1.cross != gm1.cross){
                    state = State.SHOOT;
                    timer.reset();
                }
                break;

            case SHOOT:

                Hood.state = Hood.State.SHOOT;
                spin.setPower(Odo.power);

                if (timer.seconds()>0.5){
                    state = State.RESET;
                    timer.reset();
                }
                break;

            case RESET:
                target = resetPos;
                nrBalls = 0;
                pid.setPID(Kp,0,Kd);
                Hood.state = Hood.State.IDLE;

                if (!IsStorageSpinning()){
                    Latch.state = Latch.State.IDLE;
                }
                if (!IsStorageSpinning() && Latch.state == Latch.State.IDLE) {
                    state = State.BALL1;
                }
                break;

        }
    }
    public void update(){
        spinUpdate();
        stateUpdate();
        if (state == State.TRANSFER && gm1.cross && prevgm1.cross != gm1.cross){
            state = State.SHOOT;
            timer.reset();

        }
        if (gm1.circle && prevgm1.circle!= gm1.circle && nrBalls>=1){
            state = State.TRANSFER;
        }
    }
    public void spinUpdate(){
        if (state == State.SHOOT){
            pid.setPID(0,0,0);
        }
        else {
                pid.setPID(Kp,0,Kd);
                spin.setPower(pid.calculate(FromVtoRads(), target) + Ks * Math.signum(target-FromVtoRads()));
        }
        }
    public static double FromVtoRads(){
        return Math.abs(encoder.getVoltage() / 3.3) *2.0 * Math.PI;
    }
    public static boolean IsStorageSpinning(){
        return Math.abs(target-FromVtoRads()) > Math.toRadians(11);
    }

}