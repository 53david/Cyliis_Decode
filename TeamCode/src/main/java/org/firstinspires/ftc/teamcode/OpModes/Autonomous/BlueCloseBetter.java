package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.CloseBlueBetter;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

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
