package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueBetter;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@Autonomous
public class BlueCloseBetter extends LinearOpMode {
    public CloseBlueBetter closeBlueBetter;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.isAutonomousActive = true;
        closeBlueBetter = new CloseBlueBetter(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            closeBlueBetter.update();
        }
    }
}
