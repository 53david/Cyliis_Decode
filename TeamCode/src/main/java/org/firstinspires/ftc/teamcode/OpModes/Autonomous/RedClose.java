package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.CloseBlue;

@Autonomous
public class RedClose extends LinearOpMode {
    CloseBlue closeRed;
    public void runOpMode() throws InterruptedException{
        isAutonomousActive = true;
        closeRed = new CloseBlue(hardwareMap);
        while (opModeIsActive()){
            closeRed.update();
        }
    }
}
