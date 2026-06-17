package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.CloseRedBetter;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@Autonomous
public class RedCloseBetter extends LinearOpMode {
    CloseRedBetter closeRedBetter;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.isAutonomousActive = true;
        closeRedBetter = new CloseRedBetter(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            closeRedBetter.update();
        }
    }
}
