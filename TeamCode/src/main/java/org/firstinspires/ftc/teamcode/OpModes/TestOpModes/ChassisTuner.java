package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
@Configurable
@TeleOp
public class ChassisTuner extends LinearOpMode {
    Chassis chassis;
    Odo odo;
    @Override
    public void runOpMode() throws InterruptedException{
        Hardware.init(hardwareMap);
        chassis = new Chassis(Chassis.State.PID);
        odo = new Odo();
        odo.reset();
        waitForStart();
        while(opModeIsActive()){
            chassis.update();
            odo.update();
            chassis.setTargetPosition(0,0,0);
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("Heading",Odo.getHeading());
            telemetry.update();
        }
    }
}