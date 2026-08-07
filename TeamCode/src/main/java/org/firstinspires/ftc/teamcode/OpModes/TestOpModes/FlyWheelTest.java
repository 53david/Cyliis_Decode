package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
@Config
@TeleOp
public class FlyWheelTest extends LinearOpMode {
    Shooter shooter;
    Odo odo;
    Shooter.State state = Shooter.State.ACTIVE;
    public static double power =0 ;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        shooter = new Shooter(Shooter.State.ACTIVE);
        odo = new Odo();
        waitForStart();
        while (opModeIsActive()){
            shooter.update();
            odo.update();
            shooter.setState(state);
            telemetry.addData("current velocity",shooter.flyWheel.getVelocity());
            telemetry.addData("target",shooter.flyWheel.getTargetVelocity());
            telemetry.addData("distance",Odo.delta);
        }
    }

}
