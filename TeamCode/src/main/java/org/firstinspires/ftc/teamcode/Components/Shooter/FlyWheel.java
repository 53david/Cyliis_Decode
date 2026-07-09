package org.firstinspires.ftc.teamcode.Components.Shooter;


import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.Voltage;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontLeft;

import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Kd;
import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Ka;
import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Ki;
import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Kp;
import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Ks;
import static org.firstinspires.ftc.teamcode.Math.ShooterCalculator.Kv;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;

@Configurable
public class FlyWheel {
    PIDController controller = new PIDController(Kp,Ki,Kd);
    public enum State{
        IDLE,
        SHOOT,
    }
    public static double errorThreshold = 80;
    public static State state = State.SHOOT;
    public static double vel = 0;
    public static double rpm = 0;
    public FlyWheel(){
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void updateState(){
        switch (state){
            case IDLE :
                vel = 450;
                break;
            case SHOOT:
                vel = ShooterCalculator.fwVel(Odo.distance());
                break;
        }
    }
    public void update(){
        controller.setPID(Kp,0,Kd);
        updateState();
        updateShooter();

    }
    public void updateShooter(){
        rpm = controller.calculate(getVelocity(),vel) + Kv * vel
                + Ks * Math.signum(vel-getVelocity()) + (vel-getVelocity()) * Ka;
        rpm = rpm * Voltage;
        shoot1.setPower(rpm);
        shoot2.setPower(rpm);

    }
    public static boolean isReady(){
        return Math.abs(vel-getVelocity()) < errorThreshold;
    }
    public static double getVelocity(){
        return Math.abs(frontLeft.getVelocity());
    }

}