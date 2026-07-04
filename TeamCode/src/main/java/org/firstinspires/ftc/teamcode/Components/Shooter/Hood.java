package org.firstinspires.ftc.teamcode.Components.Shooter;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.hood;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class Hood {
    public static double pos = 0.3;
    public static double IdlePos = 0.5;
    public enum State{
        IDLE,
        SHOOT,
    }
    public static State state = State.IDLE;
    public void updateState(){
        switch (state){
            case IDLE :
                hood.setPosition(ShooterCalculator.hoodAngle(Odo.distance()));
                break;
            case SHOOT:
                hood.setPosition(ShooterCalculator.hoodAngle(Odo.distance()) - (FlyWheel.vel - FlyWheel.getVelocity())*ShooterCalculator.regression);
                break;
        }
    }
    public Hood(){
        hood.setDirection(Servo.Direction.FORWARD);
    }
    public void update(){
        updateState();
    }
    public void tune(){
        hood.setPosition(pos);
    }
}
