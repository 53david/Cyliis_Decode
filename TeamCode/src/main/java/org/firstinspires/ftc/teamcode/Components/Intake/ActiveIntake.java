package org.firstinspires.ftc.teamcode.Components.Intake;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.intakeMotor;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Wrappers.Initializer;


@Configurable
public class ActiveIntake {
    public enum State{
        IDLE,
        INTAKE,
        REVERSE,
    }
    public static State state;
    public ActiveIntake() {
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        state = State.IDLE;

    }
    public void stateUpdate() {
        switch (state){
            case INTAKE:
                intakeMotor.setPower(1);
                break;
            case IDLE:
                intakeMotor.setPower(0);
                break;
            case REVERSE:
                intakeMotor.setPower(-1);
                break;
        }
    }
    public void update(){
        stateUpdate();
        if (!Initializer.isAutonomousActive){
            if (gm1.right_bumper){
                if (Storage.state == Storage.State.TRANSFER && !Storage.IsStorageSpinning()){
                    Intake.state = Intake.State.REVERSE;
                }
                else {
                    Intake.state = Intake.State.INTAKE;
                }
            }
            else if (gm1.left_bumper){
                Intake.state = Intake.State.REVERSE;
            }
            else {
                Intake.state =Intake.State.IDLE;
            }
        }
    }
    public void test(double x){
        intakeMotor.setPower(x);
    }

}