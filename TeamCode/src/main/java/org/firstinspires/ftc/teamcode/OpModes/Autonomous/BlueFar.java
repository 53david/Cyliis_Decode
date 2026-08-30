package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.LogicNodes.FarBlue;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous(name="Sper_ca_merge-BLUE - far")
public class BlueFar extends LinearOpMode {
    VoltageSensor voltageSensor;
    public FarBlue farBlue;
    @Override
    public void runOpMode() throws InterruptedException{
        Chassis.stop = false;
        farBlue = new FarBlue(hardwareMap);
        Turret.offsetY = -22;
        while (opModeInInit()){
            farBlue.intake.update();
            farBlue.timer.reset();
            farBlue.odo.update();
            farBlue.shooter.turret.update();
            farBlue.globalTimer.reset();
        }
        waitForStart();

        while(opModeIsActive()){
            farBlue.update();
            telemetry.addData("state",farBlue.shooter.flyWheel.getState());
            telemetry.addData("target",farBlue.shooter.flyWheel.getTargetVelocity());
            telemetry.addData("currentVel",farBlue.shooter.flyWheel.getVelocity());
            telemetry.addData("X", Odo.predictedX);
            telemetry.addData("Y", Odo.predictedX);
            telemetry.addData("H", Odo.heading);

            telemetry.update();
        }
    }
}
