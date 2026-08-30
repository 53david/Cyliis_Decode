package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseRed;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous(name="Sper_ca_merge-RED")
public class RedClose extends LinearOpMode {
    VoltageSensor voltageSensor;
    public CloseRed closeRed;
    @Override
    public void runOpMode() throws InterruptedException{
        Turret.offsetY = 0;
        Chassis.stop = false;
        closeRed = new CloseRed(hardwareMap);
        while (opModeInInit()){
            closeRed.intake.update();
            closeRed.odo.update();
            closeRed.shooter.turret.update();
            closeRed.globalTimer.reset();
        }
        waitForStart();

        while(opModeIsActive()){
            closeRed.update();
            telemetry.addData("Current Node",closeRed.currentNode.getName());
            telemetry.addData("Current Node index",closeRed.currentNode.index);
            telemetry.addData("Chassis stop", Chassis.stop);
            telemetry.addData("state",closeRed.shooter.flyWheel.getState());
            telemetry.addData("target",closeRed.shooter.flyWheel.getTargetVelocity());
            telemetry.addData("currentVel",closeRed.shooter.flyWheel.getVelocity());
            telemetry.addData("X", Odo.predictedX);
            telemetry.addData("Y", Odo.predictedX);
            telemetry.addData("H", Odo.heading);
            telemetry.update();
        }
    }
}
