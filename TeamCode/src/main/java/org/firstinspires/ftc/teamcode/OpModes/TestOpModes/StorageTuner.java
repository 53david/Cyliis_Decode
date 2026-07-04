package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Components.Intake.ColorDetection;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
@Configurable
public class StorageTuner extends LinearOpMode {
    public static double x =0;
    FlyWheel flyWheel;
    Storage storage;
    Latch latch;
    ColorDetection colorDetection;
    ActiveIntake activeIntake;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.start(hardwareMap);
        latch = new Latch();
        flyWheel = new FlyWheel();
        storage = new Storage();
        colorDetection = new ColorDetection();
        activeIntake = new ActiveIntake();
        waitForStart();
        while (opModeIsActive()) {
            activeIntake.test(x);
            storage.update();
            latch.update();
            colorDetection.update();
            telemetryM.addData("Target pos",Math.toDegrees(Storage.target));
            telemetryM.update();
        }
    }
}
