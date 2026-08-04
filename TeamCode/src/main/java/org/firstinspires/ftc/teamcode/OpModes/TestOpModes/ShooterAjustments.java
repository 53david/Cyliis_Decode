package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

import java.io.CharArrayReader;

@TeleOp
@Configurable
public class ShooterAjustments extends LinearOpMode {
    FlyWheel flyWheel;
    Chassis chassis;
    Hood hood;
    Odo odo;
    @Override
    public void runOpMode() throws InterruptedException{
            Hardware.init(hardwareMap);
           flyWheel = new FlyWheel();
           hood = new Hood();
           odo = new Odo();
           chassis = new Chassis(Chassis.State.DRIVE);
           FlyWheel.state = FlyWheel.State.IDLE;
           waitForStart();
           while (opModeIsActive()){
               flyWheel.update();
               odo.update();

               telemetry.addData("Delta",Odo.distance());
               telemetry.addData("Velocity",flyWheel.getVelocity());
               telemetry.update();
           }
    }
}
