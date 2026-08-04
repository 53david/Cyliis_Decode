package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseRed;
@Autonomous(name="Sper_ca_merge-RED")
public class RedClose extends LinearOpMode {
    CloseRed closeRed;
    @Override
    public void runOpMode() throws InterruptedException{
        closeRed = new CloseRed(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            Chassis.state = Chassis.State.PID;
            closeRed.update();
        }
    }
}
