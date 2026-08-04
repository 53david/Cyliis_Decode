package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Trajectories.FarBlue;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
@Configurable
@TeleOp
public class LocalizerTest extends LinearOpMode {
    Odo odo;
    @Override
    public void runOpMode()throws InterruptedException{
        Hardware.init(hardwareMap);
        odo = new Odo();
        odo.reset();
        waitForStart();
        while (opModeIsActive()){
            odo.update();
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("offset x", Turret.tx*Math.cos(Odo.getHeading()));
            telemetry.addData("offset y", Turret.tx*Math.sin(Odo.getHeading()));
            telemetry.addData("Heading - Deg",Math.toDegrees(Odo.getHeading()));
            telemetry.addData("Heading - Rads",Odo.getHeading());
            telemetry.update();
        }
    }
}
