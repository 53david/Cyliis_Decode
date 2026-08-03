package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;

public class BetterMotor {
    public enum State{
        PID,
        NORMAL
    };
    double kp = 0, ki = 0, kd = 0, kv = 0, ks = 0, ka = 0;
    int x=0;
    State state;
    public DcMotorEx motor;
    DcMotorEx encoder;
    public double currentPos = 0;
    public double targetPos = 0;
    public double power = 0;
    PIDController pid = new PIDController(kp,ki,kd);
    public BetterMotor(DcMotorEx motor,State state){
        this.motor = motor;
        this.state = state;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public BetterMotor(DcMotorEx motor,State state,boolean reversed){
        this.motor = motor;
        this.state = state;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (reversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }
    public BetterMotor(DcMotorEx motor,State state,boolean reversed,DcMotorEx encoder){
        this.motor = motor;
        this.state = state;
        this.encoder = encoder;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (reversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }
    public BetterMotor(DcMotorEx motor,State state,boolean reversed,DcMotorEx encoder,boolean encoderReversed){
        this.motor = motor;
        this.state = state;
        this.encoder = encoder;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (reversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (encoderReversed){
            this.x = -1;
        }

    }
    public void setPidCoefficients(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public void setFeedForwardCoefficients(double ks,double kv, double ka){
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }
    public void setPower(double power){
        this.power = power;
    }
    public void setCurrentPos(double a){
        currentPos = a;
    }
    public double getCurrentVelocity(){
        return encoder.getVelocity() * x;
    }
    public int getEncoderPos(){
        return encoder.getCurrentPosition() * x;
    }
    public void setTargetPos(double a){
        targetPos = a;
    }
    public void update(){
        if (state == State.NORMAL){
            motor.setPower(power);
        }
        else if (state == State.PID){
            pid.kp = kp;
            pid.ki = ki;
            pid.kd = kd;
            motor.setPower(pid.calculate(targetPos,currentPos)
                    + ks * Math.signum(targetPos-currentPos)+ kv*targetPos + ka * (targetPos- currentPos));
        }
    }
}
