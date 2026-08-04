package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LogicNodes.FarBlue;


@Autonomous
public class BlueFar extends LinearOpMode {
    FarBlue farBlue;
    @Override
    public void runOpMode(){
        farBlue = new FarBlue(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            farBlue.update();
        }
    }
}
