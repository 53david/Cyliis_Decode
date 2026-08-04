package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class VelocityTest extends LinearOpMode {
    Odo odo;
    @Override
    public void runOpMode(){
        odo = new Odo();
        waitForStart();
        while (opModeIsActive()){
            odo.update();
            telemetry.addData("Xvelocity",Odo.xRobotVelocity);
            telemetry.addData("Yvelocity",Odo.yRobotVelocity);
            telemetry.update();
        }
    }
}
