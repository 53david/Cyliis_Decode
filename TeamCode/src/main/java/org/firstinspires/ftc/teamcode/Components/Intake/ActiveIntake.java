package org.firstinspires.ftc.teamcode.Components.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;


@Config
public class ActiveIntake {
    public static double idlePower = 0,intakePower = 1, reversePower = -0.5,shootPower = 1;
    CRServo motor;
    public enum State{
        IDLE(idlePower),
        INTAKE(intakePower),
        REVERSE(reversePower),
        SHOOT(shootPower);
        double power;
        State(double power){
            this.power = power;
        }
    }
    public State state = State.IDLE;
    public ActiveIntake() {
        motor= Hardware.sch0;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void updatePower(){
        State.IDLE.power = idlePower;
        State.INTAKE.power = intakePower;
        State.REVERSE.power = reversePower;
        State.SHOOT.power = shootPower;
    }

    private void updateHardware()
    {
        motor.setPower(state.power);
    }

    public void update(){

        updateHardware();
        updatePower();
    }
    public void setState(State state){
        this.state = state;
    }
}