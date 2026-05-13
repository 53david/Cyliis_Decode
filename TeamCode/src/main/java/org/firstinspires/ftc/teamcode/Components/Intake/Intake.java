package org.firstinspires.ftc.teamcode.Components.Intake;


public class Intake {
    public ColorDetection colorDetection;
    public ActiveIntake activeIntake;
    public Storage storage;
    public Latch latch;

    public enum State{
        IDLE,
        REVERSE,
        INTAKE,
    }
    public static State state;
    public Intake(){
        state = State.IDLE;
        colorDetection = new ColorDetection();
        activeIntake = new ActiveIntake();
        storage = new Storage();
        latch = new Latch();
    }
    public void update(){
        stateUpdate();
        latch.update();
        storage.update();
        activeIntake.update();
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
        }
    }
}
