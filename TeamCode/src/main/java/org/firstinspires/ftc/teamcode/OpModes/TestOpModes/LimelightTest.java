package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;


@TeleOp
public class LimelightTest extends LinearOpMode {
    public LimeLight limeLight;

    public void runOpMode() {
        limeLight = new LimeLight();
        waitForStart();
        while (opModeIsActive()) {
            limeLight.update();
            if (LimeLight.isActive()) {
                telemetryM.addData("Target heading", LimeLight.getHeading());
                telemetryM.addData("Distance", LimeLight.getDistance());
                telemetryM.addData("Area", LimeLight.getArea());
            } else {
                telemetryM.addLine("Waiting for stream..");

            }
            telemetryM.update();
        }
    }
}