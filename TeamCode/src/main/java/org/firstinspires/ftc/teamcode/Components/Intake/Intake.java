package org.firstinspires.ftc.teamcode.Components.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

public class Intake {
    ElapsedTime timer;
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
        timer = new ElapsedTime();
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
        if (storage.getState() == Storage.State.GOINGBALL1 || storage.state == Storage.State.BALL1)latch.setState(Latch.State.GOINGIDLE);
    }
    public void updateState(){
        switch (state){
            case IDLE:
                timer.reset();
                activeIntake.setState(ActiveIntake.State.IDLE);
                break;
            case INTAKE:
                timer.reset();
                if (isBallInStorage())storage.goNext();
                activeIntake.setState(ActiveIntake.State.INTAKE);
                break;
            case REVERSE:
                timer.reset();
                activeIntake.setState(ActiveIntake.State.REVERSE);
                break;
            case SHOOT:
                storage.setState(Storage.State.SHOOT);
                activeIntake.setState(ActiveIntake.State.SHOOT);
                if (storage.state == Storage.State.GOINGBALL1 || storage.state == Storage.State.BALL1 || timer.seconds()>0.4) {state = State.IDLE; }
                break;
        }

    }
    public void setState(State state){
        this.state = state;
    }
    public State getState(){
        return state;
    }
    public boolean isBallInStorage(){
        return !bb.getState();
    }
}
