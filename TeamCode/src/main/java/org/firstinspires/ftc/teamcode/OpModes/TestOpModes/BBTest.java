package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

import java.io.CharArrayReader;

@TeleOp
public class BBTest extends LinearOpMode {
    Intake intake;

    @Override
    public void runOpMode()throws InterruptedException{
        Hardware.init(hardwareMap);
        intake = new Intake();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Is Object Nearby?",!Hardware.bb.getState());
            telemetry.addData("Is Object Nearby?", Intake.isBallInStorage());
            telemetry.update();
        }
    }
}
