package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
@Configurable
@TeleOp
public class ChassisTuner extends LinearOpMode {
    Chassis chassis;
    Odo odo;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.start(hardwareMap);
        chassis = new Chassis(Chassis.State.PID);
        odo = new Odo();
        odo.reset();
        waitForStart();
        while(opModeIsActive()){
            chassis.update();
            odo.update();
            chassis.setTargetPosition(0,0,0);
            telemetryM.addData("X",Odo.getX());
            telemetryM.addData("Y",Odo.getY());
            telemetryM.addData("Heading",Odo.getHeading());
            telemetryM.update();
        }
    }
}