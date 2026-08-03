package org.firstinspires.ftc.teamcode.Components.Chassis;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.backLeft;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.backRight;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontLeft;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontRight;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.pp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
@Configurable
public class Chassis{

    public enum State{
        DRIVE , PID;
    }
    public static State state;
    public static boolean stop = false;
    public  double targetX , targetY ,x=0 ,y=0;
    public static double targetHeading;
    public static double error;
    double rotation,heading;
    public boolean usingTargetHeading=true;

    public static double lateralMultiplier=1.5;
    public static  double realHeading;

    public static double kp=0.0065 , kd=0;
    public static double KP=1.25 , KD=0.15;
    public PIDController controllerX=new PIDController(kp, 0, kd);
    public PIDController controllerY=new PIDController(kp, 0, kd);
    public PIDController controllerHeading=new PIDController(KP, 0, KD);

    public Chassis(State initialState)
    {
        state=initialState;
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public boolean inPosition(double ErrorX , double ErrorY , double ErrorRx)
    { 
        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        error=targetHeading-realHeading;
        if(Math.abs(error)>Math.PI)
            error=-Math.signum(error)*(2*Math.PI-Math.abs(error));

        if(Math.abs(targetX-pp.getPosX(DistanceUnit.MM))<ErrorX
                && Math.abs(targetY-pp.getPosY(DistanceUnit.MM))<ErrorY
                        && Math.abs(error)<ErrorRx)
            return true;
        return false;
    }

    public void setTargetVector(double x , double y , double rx)
    {
        if (stop) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            return;
        }
        x*=lateralMultiplier;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) , 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

    public void setMode(State state)
    {
        Chassis.state =state;
    }

    public void setTargetPosition(double x , double y , double heading)
    {
        targetX=x;
        targetY=y;
        targetHeading=heading-Math.floor((heading/ (Math.PI*2)))*Math.PI*2;
        usingTargetHeading=true;

    }

    public void setTargetSpecialPosition(double targetX , double targetY , double targetHeading)
    {
        this.targetX=targetX;
        this.targetY=targetY;
        rotation=targetHeading;
        usingTargetHeading=false;
    }
    public void setTargetSpecialPosition(Pose2D position)
    {
        this.targetX=position.x;
        this.targetY=position.y;
        rotation=targetHeading;
        usingTargetHeading=false;
    }
    public void setTargetPosition(Pose2D position)
    {
        targetX=position.x;
        targetY=position.y;
        targetHeading=position.heading;
        usingTargetHeading=true;
    }
    public void updatePID(){
        controllerX.kp=kp;
        controllerY.kp=kp;

        controllerX.kd=kd;
        controllerY.kd=kd;

        controllerHeading.kp=KP;
        controllerHeading.kd=KD;
    }

    public void update() {
        updatePID();
        if (state!=State.PID){
            return;
        }
        if (Double.isNaN(Odo.x) || Double.isNaN(Odo.y) || Double.isNaN(Odo.heading)) {
            return;
        }
        x = controllerX.calculate(targetX, Odo.predictedX);
        y = -controllerY.calculate(targetY, Odo.predictedY);

        heading = Odo.getHeading();
        if (heading < 0) realHeading = Math.abs(heading);
        else realHeading = 2 * Math.PI - heading;

        error = targetHeading - realHeading;
        if (Math.abs(error) > Math.PI) {
            error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
        }
        rotation = controllerHeading.calculate(error, 0.0);

        setTargetVector(y * Math.cos(-heading) - x * Math.sin(-heading), y * Math.sin(-heading) + x * Math.cos(-heading), rotation);
    }

}