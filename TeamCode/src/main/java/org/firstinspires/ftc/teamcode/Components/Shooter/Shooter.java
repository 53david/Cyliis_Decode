package org.firstinspires.ftc.teamcode.Components.Shooter;

public class Shooter {
    public FlyWheel flyWheel;
    public Hood hood;
    public Turret turret;
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
            case IDLE:
                flyWheel.setState(FlyWheel.State.IDLE);
                hood.setState(Hood.State.IDLE);
                break;
            case ACTIVE:
                flyWheel.setState(FlyWheel.State.SHOOT);
                hood.setState(Hood.State.IDLE);
                break;
            case SHOOT:
                flyWheel.setState(FlyWheel.State.SHOOT);
                hood.setState(Hood.State.SHOOT);
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
