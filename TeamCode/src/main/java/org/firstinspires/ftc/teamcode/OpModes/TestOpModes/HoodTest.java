package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class HoodTest extends LinearOpMode {
    Hood hood;
    Odo odo;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = new Odo();
        hood = new Hood();
        while (opModeIsActive()){
            odo.update();
            hood.update();
            telemetry.addData("Distance",Odo.delta);
            telemetry.addData("Pos",hood.getPosition());
            telemetry.update();
        }
    }
}
