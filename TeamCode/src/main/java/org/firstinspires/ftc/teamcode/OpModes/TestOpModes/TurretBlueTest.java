package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.BlueClose;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
@Config
public class TurretBlueTest extends LinearOpMode {
    Turret turret;
    Odo odo;
    @Override
    public void runOpMode() throws InterruptedException{
        Hardware.init(hardwareMap);
        turret = new Turret();
        turret.setState(Turret.State.BLUE);
        odo = new Odo();
        waitForStart();
        while (opModeIsActive()){
            turret.update();
            odo.update();
        }
    }

}
