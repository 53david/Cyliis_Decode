package org.firstinspires.ftc.teamcode.Wrappers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.pp;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Math.LowPassFilter;
import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;

import java.lang.Math;

@Configurable
public class Odo {
    public enum State{
        CLOSE,
        FAR,
    }
    public static double power = -1;
    public static State state = State.CLOSE;
    public  static double heading,x ,y, xVelocity, yVelocity, predictedX, predictedY;
    public Odo(){
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED , org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setOffsets(129.503 , -78.001, MM);
    }

    public static double getHeading() {
        return heading;
    }
    public static double getX(){
        return predictedX;
    }
    public static double getY() {
        return predictedY;
    }
    public static double getRawX(){
        return pp.getPosX(MM);
    }
    public static double getRawY(){
        return pp.getPosY(MM);
    }
    public static double velX(){
        return pp.getVelX(MM);
    }
    public static double velY(){
        return pp.getVelY(MM);
    }
    public void reset() {
        pp.setPosition(new Pose2D(MM , 0 , 0 , RADIANS , 0));
    }
    public void recalibrate(){
        pp.recalibrateIMU();
    }
    public static double filterParameter = 0.8;
    private static final LowPassFilter xVelocityFilter = new LowPassFilter(filterParameter, 0);
    private static final LowPassFilter yVelocityFilter = new LowPassFilter(filterParameter, 0);


    public static double xDeceleration = 100 * 20 , yDeceleration = 150 * 20;
    public static double xRobotVelocity, yRobotVelocity;
    public static double forwardGlide, lateralGlide;
    public static double xGlide, yGlide;


    private static void updateGlide(){

        xRobotVelocity = xVelocity * Math.cos(-heading) - yVelocity * Math.sin(-heading);
        yRobotVelocity = xVelocity * Math.sin(-heading) + yVelocity * Math.cos(-heading);

        forwardGlide = Math.signum(xRobotVelocity) * xRobotVelocity * xRobotVelocity / (2.0 * xDeceleration);
        lateralGlide = Math.signum(yRobotVelocity) * yRobotVelocity * yRobotVelocity / (2.0 * yDeceleration);

        xGlide = forwardGlide * Math.cos(heading) - lateralGlide * Math.sin(heading);
        yGlide = forwardGlide * Math.sin(heading) + lateralGlide * Math.cos(heading);
    }

    public static double distance(){
        return Math.sqrt(
                (predictedX - Turret.goalPositionX) * (predictedX - Turret.goalPositionX) +
                        (predictedY - Turret.goalPositionY) * (predictedY - Turret.goalPositionY)
        );
    }
    public static double rawDistance(){
        return Math.sqrt(
                (pp.getPosX(DistanceUnit.MM) - Turret.goalPositionX) * (pp.getPosX(DistanceUnit.MM) - Turret.goalPositionX) +
                        (pp.getPosY(DistanceUnit.MM) - Turret.goalPositionY) * (pp.getPosY(DistanceUnit.MM) - Turret.goalPositionY)
        );
    }
    public static double avgVel(){
        return Math.hypot(pp.getVelX(MM),pp.getVelY(MM));
    }
    public void stateUpdate(){
        switch (state){
            case FAR :
                power = -0.9;
                Turret.angleOffset = 4;
                break;
            case CLOSE:
                power = -1;
                Turret.angleOffset = 0;
                break;
        }
        if (distance()>2800){
            state = State.FAR;
        }
        else {
            state = State.CLOSE;
        }
    }
    public void update() {
        pp.update();
        heading=pp.getHeading(RADIANS);
        x=pp.getPosX(MM);
        y=pp.getPosY(MM);
        xVelocity = xVelocityFilter.getValue(pp.getVelocity().getX(MM));
        yVelocity = yVelocityFilter.getValue(pp.getVelocity().getY(MM));
        updateGlide();
        stateUpdate();
        predictedX = x + xGlide;
        predictedY = y + yGlide;
        ShooterCalculator.updateTrajectory(pp.getPosX(MM),pp.getPosY(MM),pp.getVelX(MM),pp.getVelY(MM),Turret.goalPositionX,Turret.goalPositionY);
    }
}