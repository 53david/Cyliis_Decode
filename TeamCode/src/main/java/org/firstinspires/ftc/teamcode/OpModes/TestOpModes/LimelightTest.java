package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;


@TeleOp
public class LimelightTest extends LinearOpMode {
    public LimeLight limeLight;
    public static int index = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.start(hardwareMap);
        limeLight = new LimeLight();
        LimeLight.streamState = LimeLight.StreamState.STREAM;
        waitForStart();
        while (opModeIsActive()) {
            limeLight.update();
            limeLight.setIndex(index);
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