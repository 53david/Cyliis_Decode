package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@TeleOp
@Config
public class StorageTuner extends LinearOpMode {
    public static double x =0;
    FlyWheel flyWheel;
    Storage storage;
    Latch latch;
    ActiveIntake activeIntake;
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        latch = new Latch();
        flyWheel = new FlyWheel();
        storage = new Storage();
        activeIntake = new ActiveIntake();
        waitForStart();
        while (opModeIsActive()) {
            storage.update();
            latch.update();
            telemetry.addData("Current pos",Math.toDegrees(Storage.angle));
            telemetry.addData("Is ball in storage", Intake.isBallInStorage());
            telemetry.update();
        }
    }
}
