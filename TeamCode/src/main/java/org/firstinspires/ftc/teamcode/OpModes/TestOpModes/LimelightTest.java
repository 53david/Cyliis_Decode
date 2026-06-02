package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Vision;


@TeleOp
public class LimelightTest extends LinearOpMode {
    public Vision vision;

    public void runOpMode() {
        vision = new Vision();
        waitForStart();
        while (opModeIsActive()) {
            vision.update();
            if (Vision.isActive()) {
                telemetryM.addData("Target heading", Vision.getHeading());
                telemetryM.addData("Distance", Vision.getDistance());
                telemetryM.addData("Area", Vision.getArea());
            } else {
                telemetryM.addLine("Waiting for stream..");

            }
            telemetryM.update();
        }
    }
}