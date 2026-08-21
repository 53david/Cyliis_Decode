package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.slf4j.impl.HandroidLoggerAdapter;

@Config
@TeleOp
public class ServoCalibration extends LinearOpMode {
    public static double position=0;
    @Override
    public void runOpMode()throws InterruptedException{
        Hardware.init(hardwareMap);
        Hardware.ssh1.setPwmRange(new PwmControl.PwmRange(500,2500));
        Hardware.ssh2.setPwmRange(new PwmControl.PwmRange(500,2500));
        waitForStart();
        while (opModeIsActive()){
            Hardware.ssh1.setPosition(position);
            Hardware.ssh2.setPosition(position);

        }
    }
}
