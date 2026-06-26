package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
@Configurable
@TeleOp
public class LocalizerTest extends LinearOpMode {
    Odo odo;
    @Override
    public void runOpMode()throws InterruptedException{
        Initializer.start(hardwareMap);
        odo = new Odo();
        odo.reset();
        odo.recalibrate();
        waitForStart();
        while (opModeIsActive()){
            odo.update();
            telemetryM.addData("X",Odo.getX());
            telemetryM.addData("Y",Odo.getY());
            telemetryM.addData("Heading - Deg",Math.toDegrees(Odo.getHeading()));
            telemetryM.addData("Heading - Rads",Odo.getHeading());
            telemetryM.update();
        }
    }
}
