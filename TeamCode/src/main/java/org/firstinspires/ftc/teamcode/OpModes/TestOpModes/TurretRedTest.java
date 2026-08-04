package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@TeleOp
@Config
public class TurretRedTest extends LinearOpMode {
    Turret turret;
    @Override
    public void runOpMode() throws InterruptedException{
        Hardware.init(hardwareMap);
        turret = new Turret();
        turret.setState(Turret.State.RED);
        waitForStart();
        while (opModeIsActive()){
            turret.update();
        }
    }

}
