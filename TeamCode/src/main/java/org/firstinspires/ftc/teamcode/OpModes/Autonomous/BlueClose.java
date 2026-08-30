package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous(name="Sper_ca_merge-BLUE")
public class BlueClose extends LinearOpMode {
    VoltageSensor voltageSensor;
    public CloseBlue closeBlue;
    @Override
    public void runOpMode() throws InterruptedException{
        Turret.offsetY = 0;
        Chassis.stop = false;
        closeBlue = new CloseBlue(hardwareMap);
        while (opModeInInit()){
            closeBlue.intake.update();
            closeBlue.odo.update();
            closeBlue.shooter.turret.update();
            closeBlue.globalTimer.reset();
        }
        waitForStart();

        while(opModeIsActive()){
            closeBlue.update();
            telemetry.addData("state",closeBlue.shooter.flyWheel.getState());
            telemetry.addData("target",closeBlue.shooter.flyWheel.getTargetVelocity());
            telemetry.addData("currentVel",closeBlue.shooter.flyWheel.getVelocity());
            telemetry.addData("X", Odo.predictedX);
            telemetry.addData("Y", Odo.predictedX);
            telemetry.addData("H", Odo.heading);

            telemetry.update();
        }
    }
}
