package org.firstinspires.ftc.teamcode.Components.Shooter;


import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.Voltage;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.shoot2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontLeft;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Configurable
public class FlyWheel {
    PIDController controller = new PIDController(Kp,Ki,Kd);
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Ks = 0;
    public static double Kv = 0.000435;
    public static double Ka = 0.0055;
    public int[] v = {
            1300,
            1435,
            1480,
            1520,
            1550,
            1600,
            1630,
            1700,
            1740,
            1775,
            1820,
            // sunt scoase din pula
    };
    public static double shootPower = 0,idlePower = 1200,currentVelocity = 0,targetVelocity =0;
    public enum State{
        IDLE(idlePower),
        SHOOT(shootPower);
        double power;
        State(){

        }
        State(double power){
            this.power = power;
        }

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
            case IDLE:
            case SHOOT:
                int i = Math.max((Odo.delta%100-8),0);
                i = Math.min(i,v.length-1);
                shootPower = v[i];
                break;
        }
    }
    public void update(){
        targetVelocity = state.power;
        currentVelocity = getVelocity();
        updateState();
        updateShooter();

    }
    public void updateShooter(){
        rpm = controller.calculate(currentVelocity,targetVelocity) + Kv * targetVelocity
                + Ks * Math.signum(targetVelocity- currentVelocity) + (targetVelocity-currentVelocity) * Ka;
        rpm *=Voltage;
        shoot1.setPower(rpm);
        shoot2.setPower(rpm);

    }
    public static boolean isReady(){
        return Math.abs(vel-currentVelocity) < errorThreshold;
    }
    public static double getVelocity(){
        return Math.abs(frontLeft.getVelocity());
    }

}