package org.firstinspires.ftc.teamcode.Components.Chassis;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.backLeft;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.backRight;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontLeft;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.frontRight;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.pp;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;


import com.bylazar.configurables.annotations.Configurable;
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

    public  double targetX , targetY ,x=0 ,y=0;
    public static double targetHeading;
    public static double error;
    double rotation;
    public boolean usingTargetHeading=true;

    public static double lateralMultiplier=1.5;
    public static  double realHeading;

    public static double kp=0.0095 , kd=0;
    public static double KP=1.8 , KD=0.11;
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

    public void update() {

        if (state == State.DRIVE && Turret.allianceState == Turret.AllianceState.BLUE) {

            double X = gm1.left_stick_x;
            double Y = -gm1.left_stick_y;
            double rx = (gm1.right_trigger - gm1.left_trigger);
            double heading = -Odo.getHeading() + Math.PI / 2;
            double x = X * Math.cos(heading) - Y * Math.sin(heading);
            double y = X * Math.sin(heading) + Y * Math.cos(heading);
            setTargetVector(x, y, rx);
        }
        else if (state == State.DRIVE && Turret.allianceState == Turret.AllianceState.RED) {

            double X = gm1.left_stick_x;
            double Y = -gm1.left_stick_y;
            double rx = (gm1.right_trigger - gm1.left_trigger);
            double heading = -Odo.getHeading() - Math.PI / 2;
            double x = X * Math.cos(heading) - Y * Math.sin(heading);
            double y = X * Math.sin(heading) + Y * Math.cos(heading);
            setTargetVector(x, y, rx);
        }
        else {
            controllerX.kp=kp;
            controllerY.kp=kp;

            controllerX.kd=kd;
            controllerY.kd=kd;

            controllerHeading.kp=KP;
            controllerHeading.kd=KD;
            if (Double.isNaN(Odo.x) || Double.isNaN(Odo.y) || Double.isNaN(Odo.heading)) {
                return;
            }
            x = controllerX.calculate(targetX, Odo.predictedX);
            y = -controllerY.calculate(targetY, Odo.predictedY);

            double heading = Odo.getHeading();
            if (heading < 0) realHeading = Math.abs(heading);
            else realHeading = 2 * Math.PI - heading;

            error = targetHeading - realHeading;
            if (Math.abs(error) > Math.PI)
                error = -Math.signum(error) * (2 * Math.PI - Math.abs(error));
            rotation = controllerHeading.calculate(error, 0.0);

            setTargetVector(y * Math.cos(-heading) - x * Math.sin(-heading), y * Math.sin(-heading) + x * Math.cos(-heading), rotation);

        }
    }

}