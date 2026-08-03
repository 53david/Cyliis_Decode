package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

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
            telemetryM.addData("Xvelocity",Odo.xRobotVelocity);
            telemetryM.addData("Yvelocity",Odo.yRobotVelocity);
            telemetryM.update();
        }
    }
}
