package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.pp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
public class OdoRecalibration extends LinearOpMode {
    public void runOpMode(){
        Initializer.start(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            pp.resetPosAndIMU();
            pp.recalibrateIMU();
            requestOpModeStop();
        }
    }
}
