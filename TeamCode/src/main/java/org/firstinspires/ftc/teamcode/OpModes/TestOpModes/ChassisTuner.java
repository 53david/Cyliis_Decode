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
    public void runOpMode(){
        chassis = new Chassis(Chassis.State.PID);
        waitForStart();
        while(opModeIsActive()){
            chassis.update();
            chassis.setTargetPosition(0,0,0);
        }
    }
}