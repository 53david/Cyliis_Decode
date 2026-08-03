package org.firstinspires.ftc.teamcode.Components.Intake;


import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.proximitySensor;

public class Intake {
    public ActiveIntake activeIntake;
    public Storage storage;
    public Latch latch;
    boolean ok = false;

    public enum State{
        IDLE,
        REVERSE,
        INTAKE,
        SHOOT,
    }
    public static State state;
    public Intake(){
        state = State.IDLE;
        activeIntake = new ActiveIntake();
        storage = new Storage();
        latch = new Latch();
    }
    public void update(){
        stateUpdate();
        latch.update();
        storage.update();
        activeIntake.update();
        if (Storage.state == Storage.State.TRANSFER){
            Latch.state = Latch.State.GOINGTRANSFER;
        }
        if (Storage.state == Storage.State.GOINGBALL1){
            Latch.state = Latch.State.GOINGIDLE;
        }
    }
    public void stateUpdate(){
        switch (state){
            case IDLE:
                ActiveIntake.state = ActiveIntake.State.IDLE;
                break;
            case INTAKE:
                ActiveIntake.state = ActiveIntake.State.INTAKE;
                break;
            case REVERSE:
                ActiveIntake.state = ActiveIntake.State.REVERSE;
                break;
            case SHOOT:
                Storage.shoot();
                ActiveIntake.state = ActiveIntake.State.SHOOT;
                if (Storage.state == Storage.State.GOINGBALL1) state = State.IDLE;
                break;
        }
    }
    public static boolean isBallInStorage(){
        return !proximitySensor.getState();
    }
}
