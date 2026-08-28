package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.pp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@TeleOp
public class OdoRecalibration extends LinearOpMode {
    public void runOpMode(){
        Hardware.init(hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            pp.resetPosAndIMU();
            pp.recalibrateIMU();
            Odo.offsetX = 0;
            Odo.offsetY = 0;
            pp.setPosX(0, DistanceUnit.MM);
            pp.setPosX(0, DistanceUnit.MM);
            pp.setHeading(0, AngleUnit.RADIANS);
            requestOpModeStop();
        }
    }
}
