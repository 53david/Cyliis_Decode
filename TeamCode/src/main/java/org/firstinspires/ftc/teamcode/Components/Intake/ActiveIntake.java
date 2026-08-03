package org.firstinspires.ftc.teamcode.Components.Intake;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.intakeMotor;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Configurable
public class ActiveIntake {
    public double power = 0;
    public enum State{
        IDLE(0),
        INTAKE(1),
        REVERSE(-0.5),
        SHOOT(0.5);
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
        power = state.power;
    }

}