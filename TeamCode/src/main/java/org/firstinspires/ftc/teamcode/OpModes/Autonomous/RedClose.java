package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LogicNodes.CloseRed;

@Autonomous(name = "Sper_ca_merge-RED")
public class RedClose extends LinearOpMode {
    CloseRed closeRed;
    @Override
    public void runOpMode(){
        closeRed = new CloseRed(hardwareMap);
        while(opModeInInit()){
            closeRed.intake.update();
        }
        waitForStart();
        while (opModeIsActive()){
            closeRed.update();
        }
    }
}
