package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Trajectories.HivePositions.hivePos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Limelight;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
@Config
public class LimelightTest extends LinearOpMode {
    public static int index;
    Chassis chassis;
    Odo odo;
    Limelight limelight;
    public void runOpMode(){
        Hardware.init(hardwareMap);
        chassis = new Chassis(Chassis.State.PID);
        odo = new Odo();
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = new Limelight(Limelight.State.ACTIVE);
        waitForStart();
        while (opModeIsActive()){
            limelight.update();
            limelight.setPipeLineIndex(index);
            chassis.setTargetSpecialPosition(hivePos[limelight.getPositionIndex()]);
            if (chassis.inPosition(120,120,0.1))chassis.setTargetPosition(hivePos[limelight.getPositionIndex()]);
            chassis.update();
            odo.update();
            telemetry.addData("State",limelight.getState());
            telemetry.addData("Index",limelight.getPositionIndex());
            telemetry.addData("Time",limelight.getTime());
            telemetry.addData("Hive State",limelight.getHiveState());
            telemetry.addData("Target X",hivePos[limelight.getPositionIndex()].x);
            telemetry.addData("Target Y",hivePos[limelight.getPositionIndex()].y);
            telemetry.addData("Target Heading",hivePos[limelight.getPositionIndex()].heading);
            telemetry.update();
        }
    }
}
