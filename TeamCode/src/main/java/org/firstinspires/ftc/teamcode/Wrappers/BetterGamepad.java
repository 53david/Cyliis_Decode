package org.firstinspires.ftc.teamcode.Wrappers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BetterGamepad {
    ElapsedTime timer;
    public Gamepad gm1;
    public BetterGamepad(Gamepad gamepad){
        this.gm1 = gamepad;
        timer = new ElapsedTime();
        timer.startTime();
    }
    public void copy(Gamepad gamepad){
        gm1.copy(gamepad);
    }
    public boolean wasCrossPressed(double time){
        if (!cross()){
            timer.reset();
        }
        if (timer.seconds()>time) {
            timer.reset();
            return true;
        }
        return false;
    }
    public boolean wasCrossPressed(){
        return gm1.crossWasPressed();
    }
    public boolean cross(){
        return gm1.cross;
    }
    public boolean wasCirclePressed(double time){
        if (!circle()){
            timer.reset();
        }
        if (timer.seconds()>time) {
            timer.reset();
            return true;
        }
        return false;
    }
    public boolean wasCirclePressed(){
        return gm1.crossWasPressed();
    }
    public boolean circle(){
        return gm1.circle;
    }
    public boolean wasSquarePressed(double time){
        if (!square()){
            timer.reset();
        }
        if (timer.seconds()>time) {
            timer.reset();
            return true;
        }
        return false;
    }
    public boolean wasSquarePressed(){
        return gm1.crossWasPressed();
    }
    public boolean square(){
        return gm1.square;
    }
    public boolean wasTrianglePressed(){
        return gm1.crossWasPressed();
    }
    public boolean triangle(){
        return gm1.square;
    }
    public boolean wasTrianglePressed(double time){
        if (!triangle()){
            timer.reset();
        }
        if (timer.seconds()>time) {
            timer.reset();
            return true;
        }
        return false;
    }
    public boolean getInput(){
        return gm1.cross || gm1.triangle || gm1.circle || gm1.square || gm1.dpad_up || gm1.dpad_down || gm1.dpad_left || gm1.dpad_right;
    }
    public void update(){
        if (!getInput()){
            timer.reset();
        }
    }

}
