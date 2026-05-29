package org.firstinspires.ftc.teamcode.Wrappers;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
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
        pp.setOffsets(129.503 , -78.001, DistanceUnit.MM);
        pp.resetPosAndIMU();
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
    public void reset() {
        pp.setPosition(new Pose2D(DistanceUnit.MM , 0 , 0 , RADIANS , 0));
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

    public static SparkFunOTOS.Pose2D getCurrentPosition(){
        double h = pp.getHeading(RADIANS);
        h = normalizeRadians(h);
        return new SparkFunOTOS.Pose2D(getX(),getY(), h);
    }
    public static double distance(){
        return Math.sqrt(
                (predictedX - Turret.goalPositionX) * (predictedX - Turret.goalPositionX) +
                        (predictedY - Turret.goalPositionY) * (predictedY - Turret.goalPositionY)
        );
    }
    public void stateUpdate(){
        switch (state){
            case FAR :
                power = -0.7;
                ShooterCalculator.fwOffset = 60;
                break;
            case CLOSE:
                power = -1;
                ShooterCalculator.fwOffset = 80;
                break;
        }
        if (distance()>2200){
            state = State.FAR;
        }
        else {
            state = State.CLOSE;
        }
    }
    public void update() {
        pp.update();

        heading=pp.getHeading(RADIANS);
        x=pp.getPosX(DistanceUnit.MM);
        y=pp.getPosY(DistanceUnit.MM);
        xVelocity = xVelocityFilter.getValue(pp.getVelocity().getX(DistanceUnit.MM));
        yVelocity = yVelocityFilter.getValue(pp.getVelocity().getY(DistanceUnit.MM));
        updateGlide();
        stateUpdate();
        predictedX = x + xGlide;
        predictedY = y + yGlide;
    }
    public static double trueX(){
        return pp.getPosX(DistanceUnit.MM);
    }
    public static double trueY(){
        return pp.getPosY(DistanceUnit.MM);
    }
    public static double trueHeading(){
        return pp.getHeading(UnnormalizedAngleUnit.RADIANS);
    }
}