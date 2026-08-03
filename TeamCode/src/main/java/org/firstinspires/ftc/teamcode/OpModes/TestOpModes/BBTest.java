package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.proximitySensor;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import org.firstinspires.ftc.teamcode.Components.Intake.Storage;

@TeleOp
public class BBTest extends LinearOpMode {

    @Override
    public void runOpMode()throws InterruptedException{
        waitForStart();
        while (opModeIsActive()){
            telemetryM.addData("Is Object Nearby?",!proximitySensor.getState());
            telemetryM.addData("Is Object Nearby?", Storage.isBallInStorage());
            telemetryM.update();
        }
    }
}
