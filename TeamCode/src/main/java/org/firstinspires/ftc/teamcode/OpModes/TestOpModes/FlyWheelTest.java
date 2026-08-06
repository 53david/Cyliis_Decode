package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class FlyWheelTest extends LinearOpMode {
    FlyWheel flyWheel;
    Odo odo;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        flyWheel = new FlyWheel();
        odo = new Odo();
        waitForStart();
        while (opModeIsActive()){
            flyWheel.update();
            odo.update();
            telemetry.addData("current velocity",flyWheel.getVelocity());
            telemetry.addData("target",flyWheel.getTargetVelocity());
            telemetry.addData("distance",Odo.delta);
        }
    }

}
