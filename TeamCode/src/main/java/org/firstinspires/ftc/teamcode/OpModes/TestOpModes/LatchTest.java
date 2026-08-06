package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

import android.text.method.HideReturnsTransformationMethod;

@TeleOp
public class LatchTest extends LinearOpMode {
    Latch latch;
    @Override
    public void runOpMode()throws InterruptedException{
        Hardware.init(hardwareMap);
        latch = new Latch();
        waitForStart();
        while (opModeIsActive()){
            latch.update();
        }
    }
}
