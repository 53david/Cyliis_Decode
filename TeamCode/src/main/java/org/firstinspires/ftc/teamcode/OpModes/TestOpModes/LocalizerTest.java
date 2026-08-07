package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
@TeleOp
public class LocalizerTest extends LinearOpMode {
    Odo odo;

    @Override
    public void runOpMode()throws InterruptedException{
        Hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = new Odo();
        odo.reset();
        waitForStart();
        while (opModeIsActive()){
            odo.update();
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("Heading - Deg",Math.toDegrees(Odo.getHeading()));
            telemetry.addData("Heading - Rads",Odo.getHeading());
            telemetry.addData("X velocity",Odo.xRobotVelocity);
            telemetry.addData("Y velocity",Odo.yRobotVelocity);
            telemetry.update();
        }
    }
}
