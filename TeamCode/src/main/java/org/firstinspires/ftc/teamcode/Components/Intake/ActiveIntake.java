package org.firstinspires.ftc.teamcode.Components.Intake;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.intakeMotor;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
public class ActiveIntake {
    public static double idlePower = 0,intakePower = 1, reversePower = -0.5,shootPower = 1;
    public double power = 0;
    public enum State{
        IDLE(idlePower),
        INTAKE(intakePower),
        REVERSE(reversePower),
        SHOOT(shootPower);
        double power = 0;
        State(){
        }
        State(double power){
            this.power = power;
        }
    }
    public static State state = State.IDLE;
    public ActiveIntake() {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void updatePower(){
        State.IDLE.power = idlePower;
        State.INTAKE.power = intakePower;
        State.REVERSE.power = reversePower;
        State.SHOOT.power = shootPower;
    }
    public void stateUpdate() {
        switch (state){
            case INTAKE:
            case IDLE:
            case REVERSE:
            case SHOOT:
                break;
        }
    }
    public void update(){
        stateUpdate();
        updatePower();
        power = state.power;
    }

}