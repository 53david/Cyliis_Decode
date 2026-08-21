    package org.firstinspires.ftc.teamcode.Wrappers;

    import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
    import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

    import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.pp;

    import com.bylazar.configurables.annotations.Configurable;
    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.robotcore.util.ElapsedTime;


    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
    import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
    import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
    import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
    import org.firstinspires.ftc.teamcode.Math.LowPassFilter;

    import java.lang.Math;

    @Configurable
    public class Odo {

        public static double power = -1, timerTreshold;
        public static double goalPositionX = 0,goalPositionY;
        public static double heading,x ,y, xVelocity, yVelocity, predictedX, predictedY,offsetX = 0,offsetY = 0, offset = 0,prevX = 0, prevY = 0;
        public static int delta = 0;
        public Odo(){
            pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED , org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pp.setOffsets(128.5 , -76.999+17.798, DistanceUnit.MM);

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
            return xVelocity;
        }
        public static double velY(){
            return yVelocity;
        }
        public void reset() {
            offsetX = 0; offsetY = 0;
            pp.setPosition(new Pose2D(MM,0,0,RADIANS,0));
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
            return Math.sqrt((goalPositionX - (x + Turret.tx*Math.cos(Odo.heading))) * (goalPositionX - (x + Turret.tx*Math.cos(Odo.heading)))
                    + (goalPositionY - (y+ Turret.tx*Math.sin(Odo.heading))) * (goalPositionY - (y+ Turret.tx*Math.sin(Odo.heading))));
        }
        public static double avgVel(){
            return Math.hypot(pp.getVelX(MM),pp.getVelY(MM));
        }
        public static void setPosition(org.firstinspires.ftc.teamcode.Wrappers.Pose2D pose2D){
            pp.setPosX(pose2D.x,MM);
            pp.setPosY(pose2D.y,MM);
            pp.setHeading(pose2D.heading,RADIANS);
        }
        public static void setPosition(double posX,double posY,double h){
            pp.setPosX(posX,MM);
            pp.setPosY(posY,MM);
            pp.setHeading(h,RADIANS);
        }
        public void update(){
            pp.update();
            delta = (int)distance();
            heading=pp.getHeading(RADIANS);
            x=pp.getPosX(MM) + offsetX;
            y=pp.getPosY(MM) + offsetY;
            xVelocity = xVelocityFilter.getValue(pp.getVelocity().getX(MM));
            yVelocity = yVelocityFilter.getValue(pp.getVelocity().getY(MM));
            updateGlide();
            if (Odo.delta > 3000){
                power = -0.6;
                timerTreshold = 0.7;
            }
            else {
                power = -1;
                timerTreshold = 0.4;
            }

            predictedX = x + xGlide;
            predictedY = y + yGlide;
            prevX = x;
            prevY = y;
        }
    }