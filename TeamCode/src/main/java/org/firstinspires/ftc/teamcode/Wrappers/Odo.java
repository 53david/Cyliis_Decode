    package org.firstinspires.ftc.teamcode.Wrappers;

    import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
    import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
    import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
    import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.pp;

    import com.bylazar.configurables.annotations.Configurable;


    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
    import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
    import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
    import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
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
        public static double heading,x ,y, xVelocity, yVelocity, predictedX, predictedY,offsetX = 0,offsetY = 0, offset = 0;
        ShooterCalculator shooterCalculator = new ShooterCalculator();
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
            offsetY = 0;
            offsetX = 0;
            FlyWheel.offset = 0;
            Hood.offset = 0;
            Turret.offset = 0;
            pp.setPosition(new Pose2D(MM,0,0,RADIANS,0));
        }
        public void recalibrate(){
            offsetY = 0;
            offsetX = 0;
            pp.setPosition(new Pose2D(MM,0,0,RADIANS,0));
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
        public void stateUpdate(){
            switch (state){
                case FAR :
                    power = -0.9;
                    offset = Math.toRadians(1);
                    break;
                case CLOSE:
                    power = -1;
                    offset = Math.toRadians(1);
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
            x=pp.getPosX(MM) + offsetX;
            y=pp.getPosY(MM) + offsetY;
            xVelocity = xVelocityFilter.getValue(pp.getVelocity().getX(MM));
            yVelocity = yVelocityFilter.getValue(pp.getVelocity().getY(MM));
            updateGlide();
            stateUpdate();
            predictedX = x + xGlide;
            predictedY = y + yGlide;
            shooterCalculator.updateTrajectory(predictedX,predictedY
                    ,xVelocity,yVelocity,Turret.goalPositionX,Turret.goalPositionY);
            if (LimeLight.streamState == LimeLight.StreamState.STREAM) {
                if (LimeLight.getTagAngle() != 1e9 && Storage.state != Storage.State.TRANSFER && Storage.state != Storage.State.SHOOT) {
                    pp.setPosX(LimeLight.absoluteX, MM);
                    pp.setPosY(LimeLight.absoluteY, MM);
                }
            }
        }
    }