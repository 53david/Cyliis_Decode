package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.pp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@TeleOp
public class OdoRecalibration extends LinearOpMode {
    public void runOpMode(){
        Hardware.init(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            pp.resetPosAndIMU();
            pp.recalibrateIMU();
            requestOpModeStop();
        }
    }
}
