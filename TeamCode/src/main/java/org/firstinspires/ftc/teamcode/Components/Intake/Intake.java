package org.firstinspires.ftc.teamcode.Components.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

public class Intake {
    public ActiveIntake activeIntake;
    public Storage storage;
    public Latch latch;
    DigitalChannel bb;

    public enum State{
        IDLE,
        REVERSE,
        INTAKE,
        SHOOT,
    }
    public State state;
    public Intake(){
        bb = Hardware.bb;
        state = State.IDLE;
        activeIntake = new ActiveIntake();
        storage = new Storage();
        latch = new Latch();
    }
    public void update(){
        updateState();
        latch.update();
        storage.update();
        activeIntake.update();

        if (storage.getState() == Storage.State.TRANSFER)latch.setState(Latch.State.GOINGTRANSFER);
    }
    public void updateState(){
        switch (state){
            case IDLE:
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
            case INTAKE:
                if (isBallInStorage())storage.goNext();

                activeIntake.setState(ActiveIntake.State.INTAKE);
                break;
            case REVERSE:
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case SHOOT:
                storage.shoot();
                activeIntake.setState(ActiveIntake.State.SHOOT);
                if (storage.state == Storage.State.GOINGBALL1) {state = State.IDLE; latch.setState(Latch.State.GOINGIDLE);}
                break;
        }

    }
    public void setState(State state){
        if (state != State.SHOOT) this.state = state;
    }
    public boolean isBallInStorage(){
        return !bb.getState();
    }
}
