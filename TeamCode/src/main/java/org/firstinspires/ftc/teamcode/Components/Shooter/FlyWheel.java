package org.firstinspires.ftc.teamcode.Components.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Config
public class FlyWheel {
    CRServo shoot1,shoot2;
    DcMotorEx encoder;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0;
    public static double Kv = 0.000435;
    public static double Ka = 0.0055;
    public int[] v = {
            1300,
            1435,
            1480,
            1520,
            1550,
            1600,
            1630,
            1700,
            1740,
            1775,
            1820,
    };
    public static int shootPower = 0,idlePower = 1200;
    public static double currentVelocity = 0,targetVelocity =0;
    PIDController controller = new PIDController(Kp,Ki,Kd);
    public enum State{
        IDLE(idlePower),
        SHOOT(shootPower);
        double power;
        State(double power){
            this.power = power;
        }

    }
    public static double errorThreshold = 80;
    public State state = State.SHOOT;
    public static double rpm = 0;
    public FlyWheel(){
        encoder = Hardware.mch3;
        shoot1 = Hardware.sch1;
        shoot2 = Hardware.sch2;
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void update(){
        currentVelocity = encoder.getVelocity();
        updateState();
        updatePower();
        updatePID();
        updateHardware();

    }
    private void updateState(){
        switch (state){
            case IDLE:
                break;
            case SHOOT:
                int i = Math.max((Odo.delta/100-8),0);i = Math.min(i,v.length-1);
                int j = Math.max((Odo.delta/100-8)+1,0);j = Math.min(j,v.length-1);
                shootPower =  (v[i] * (Odo.delta - (800 + i * 100)) + v[j] * ((800 + j * 100) - Odo.delta)) /100;
                break;
        }
    }
    private void updatePower(){
        State.SHOOT.power = shootPower;
        State.IDLE.power = idlePower;
    }
    private void updateHardware(){
        rpm = controller.calculate(currentVelocity, state.power) + Kv * state.power
                + Ks * Math.signum(state.power- currentVelocity) + (state.power-currentVelocity) * Ka;
        shoot1.setPower(rpm);
        shoot2.setPower(rpm);

    }
    private void updatePID(){
        controller.kp = Kp;
        controller.ki = Ki;
        controller.kd = Kd;

    }
    public boolean isReady(){
        return Math.abs(currentVelocity-state.power) < errorThreshold;
    }
    public double getVelocity(){
        return currentVelocity;
    }
    public double getTargetVelocity(){
        return state.power;
    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){
        return state;
    }

}