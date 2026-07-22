package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.transfer;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.spin;
@Configurable
@TeleOp
public class LatchTest extends LinearOpMode {
    Latch latch;
    @Override
    public void runOpMode()throws InterruptedException{
        Initializer.start(hardwareMap);
        latch = new Latch();
        waitForStart();
        while (opModeIsActive()){
            latch.update();
        }
    }
}
