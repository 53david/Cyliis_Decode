package org.firstinspires.ftc.teamcode.Components.Shooter;

public class Shooter {
    FlyWheel flyWheel;
    Hood hood;
    Turret turret;
    public enum State{
        IDLE,
        SHOOT,
        ACTIVE,
    }
    public static State state = State.SHOOT;
    public Shooter(State state1){
        state = state1;
        turret = new Turret();
        flyWheel =new FlyWheel();
        hood = new Hood();
    }
    public void updateState(){
        switch (state){
            case ACTIVE:
                FlyWheel.state = FlyWheel.State.SHOOT;
                Hood.state = Hood.State.IDLE;
                break;
            case SHOOT:
                FlyWheel.state = FlyWheel.State.SHOOT;
                Hood.state = Hood.State.SHOOT;
                break;
        }

    }
    public void update(){
        updateState();
        turret.update();
        hood.update();
        flyWheel.update();
    }
}
