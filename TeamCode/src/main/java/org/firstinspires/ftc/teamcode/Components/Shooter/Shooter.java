package org.firstinspires.ftc.teamcode.Components.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;

public class Shooter {
    public FlyWheel flyWheel;
    ElapsedTime timer;
    public Hood hood;
    public Turret turret;
    public enum State{
        IDLE,
        SHOOT,
        ACTIVE,
    }
    public State state = State.ACTIVE;
    public Shooter(State state){
        timer = new ElapsedTime();
        timer.startTime();
        this.state = state;
        turret = new Turret();
        flyWheel =new FlyWheel();
        hood = new Hood();
    }
    public void updateState(){
        switch (state){
            case IDLE:
                flyWheel.setState(FlyWheel.State.IDLE);
                hood.setState(Hood.State.PAUSE);
                timer.reset();
                break;
            case ACTIVE:
                flyWheel.setState(FlyWheel.State.SHOOT);
                hood.setState(Hood.State.IDLE);
                timer.reset();
                break;
            case SHOOT:
                flyWheel.setState(FlyWheel.State.SHOOT);
                hood.setState(Hood.State.SHOOT);
                if (timer.seconds()>0.4) state = State.ACTIVE;
                break;
        }

    }
    public void update(){
        updateState();
        turret.update();
        hood.update();
        flyWheel.update();
    }
    public void setState(State state){
        this.state=state;
    }
}
