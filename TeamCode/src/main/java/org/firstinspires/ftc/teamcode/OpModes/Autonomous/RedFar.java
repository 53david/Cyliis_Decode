package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LogicNodes.FarRed;

@Autonomous
public class RedFar extends LinearOpMode {
    public FarRed farRed;
    @Override
    public void runOpMode() throws InterruptedException{
        farRed = new FarRed(hardwareMap);
        while (opModeIsActive()){
            farRed.update();
        }
    }
}
