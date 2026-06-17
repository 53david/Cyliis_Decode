package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
public class ServoCalibration extends LinearOpMode {
    @Override
    public void runOpMode()throws InterruptedException{
        Initializer.start(hardwareMap);
        Initializer.servo1.setPwmRange(new PwmControl.PwmRange(500,2500));
        Initializer.servo2.setPwmRange(new PwmControl.PwmRange(500,2500));
        waitForStart();
        while (opModeIsActive()){
            Initializer.servo1.setPosition(0.5);
            Initializer.servo2.setPosition(0.5);
        }
    }
}
