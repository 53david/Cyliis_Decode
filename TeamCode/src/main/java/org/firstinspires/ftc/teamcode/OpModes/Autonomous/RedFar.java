package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.FarRed;
@Autonomous
public class RedFar extends LinearOpMode {
    public FarRed farRed;
    public void runOpMode() throws InterruptedException{
        isAutonomousActive = true;
        farRed = new FarRed(hardwareMap);
        while (opModeIsActive()){
            farRed.update();
        }
    }
}
