package org.firstinspires.ftc.teamcode.Components.Intake;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.color;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.proximitySensor;


import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Configurable
public class ColorDetection {
    public enum State{
        SORT,
        RAPID,
    };
    public static State state = State.RAPID;
    Storage storage = new Storage();
    public static double a = 50;
    public float red = 0;
    public float green = 0;
    public float blue = 0;
    public static String ball1 = "Waiting for artifact...", ball2 = "Waiting for artifact...", ball3 = "Waiting for artifact...",currentBall = "Waiting for artifact...";
    public double greenBall = 0, purpleBall = 0;

    public ColorDetection(){
        color.enableLed(true);
    }
    public void update() {
        if (state != State.SORT && isBallInStorage() && !storage.IsStorageSpinning()) {
            red = color.red();
            blue = color.blue();
            green = color.green();
            greenBall = distance(red, green, blue, 0, 255, 0);
            purpleBall = distance(red, green, blue, 175, 0, 175);

            if (isBallInStorage() && purpleBall <= greenBall) {
                currentBall = "Green";
            } else if (isBallInStorage() && purpleBall > greenBall) {
                currentBall = "Purple";
            }

            if (Storage.state == Storage.State.BALL1
                    && isBallInStorage() && !storage.IsStorageSpinning()) {
                ball1 = currentBall;
            }

            if (Storage.state == Storage.State.BALL2
                    && isBallInStorage() && !storage.IsStorageSpinning()) {
                ball2 = currentBall;
            }

            if (Storage.state == Storage.State.BALL3
                    && isBallInStorage() && !storage.IsStorageSpinning()) {
                ball3 = currentBall;
            }
            if (Storage.state == Storage.State.RESET) {
                ball1 = "Waiting for artifact...";
                ball2 = "Waiting for artifact...";
                ball3 = "Waiting for artifact...";
            }
        }
    }
    public static boolean isBallInStorage(){
        if (state == State.RAPID) {
            return !proximitySensor.getState() || color.getDistance(DistanceUnit.MM) < a;
        }
        else{
            return color.getDistance(DistanceUnit.MM) < a;
        }
    }
    public float distance(float r1, float g1 , float b1, float r2, float g2, float b2) {
        return (float)Math.sqrt( (r1-r2)*(r1-r2) + (b1-b2)*(b1-b2) + (g1-g2)*(g1-g2));
    }
}
