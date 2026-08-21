package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseRed;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;

@Autonomous(name="Sper_ca_merge-RED")
public class RedClose extends LinearOpMode {
    VoltageSensor voltageSensor;
    public CloseRed closeRed;
    @Override
    public void runOpMode() throws InterruptedException{
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
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
            telemetry.addData("state",closeRed.shooter.flyWheel.getState());
            telemetry.addData("target",closeRed.shooter.flyWheel.getTargetVelocity());
            telemetry.addData("currentVel",closeRed.shooter.flyWheel.getVelocity());
            telemetry.update();
        }
    }
}
