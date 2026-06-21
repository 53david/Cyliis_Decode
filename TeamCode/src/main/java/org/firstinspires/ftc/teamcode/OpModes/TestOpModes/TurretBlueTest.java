package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
@Configurable
public class TurretBlueTest extends LinearOpMode {
    Turret turret;
    Odo odo;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.start(hardwareMap);
        turret = new Turret();
        odo = new Odo();
        Turret.allianceState = Turret.AllianceState.BLUE;
        waitForStart();
        while (opModeIsActive()){
            turret.update();
            odo.update();
        }
    }

}
