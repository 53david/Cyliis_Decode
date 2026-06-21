package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@TeleOp
@Configurable
public class TurretRedTest extends LinearOpMode {
    Turret turret;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.start(hardwareMap);
        turret = new Turret();
        Turret.allianceState = Turret.AllianceState.RED;
        waitForStart();
        while (opModeIsActive()){
            turret.update();
        }
    }

}
