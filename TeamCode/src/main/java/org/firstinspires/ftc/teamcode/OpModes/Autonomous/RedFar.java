package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseRed;
import org.firstinspires.ftc.teamcode.LogicNodes.FarRed;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous(name="Sper_ca_merge-RED - far")
public class RedFar extends LinearOpMode {
    VoltageSensor voltageSensor;
    public FarRed farRed;
    @Override
    public void runOpMode() throws InterruptedException{
        Chassis.stop = false;
        farRed = new FarRed(hardwareMap);
        Turret.offsetY = 10;
        while (opModeInInit()){
            farRed.intake.update();
            farRed.odo.update();
            farRed.shooter.turret.update();
            farRed.globalTimer.reset();
        }
        waitForStart();

        while(opModeIsActive()){
            farRed.update();
            telemetry.addData("Current Node",farRed.currentNode.getName());
            telemetry.addData("Current Node index",farRed.currentNode.index);
            telemetry.addData("Chassis stop", Chassis.stop);
            telemetry.addData("state",farRed.shooter.flyWheel.getState());
            telemetry.addData("target",farRed.shooter.flyWheel.getTargetVelocity());
            telemetry.addData("currentVel",farRed.shooter.flyWheel.getVelocity());
            telemetry.addData("X", Odo.predictedX);
            telemetry.addData("Y", Odo.predictedX);
            telemetry.addData("H", Odo.heading);
            telemetry.update();
        }
    }
}
