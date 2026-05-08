package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Trajectories.CloseBlue;

@Autonomous
public class RedClose extends LinearOpMode {
    CloseBlue closeBlue;
    public void runOpMode() throws InterruptedException{
        closeBlue = new CloseBlue(hardwareMap);
        while (opModeIsActive()){
            closeBlue.update();
        }
    }
}
