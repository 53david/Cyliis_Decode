package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.gm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Hardware.prevgm2;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class TeleopBlue extends LinearOpMode {

    public static double currentVoltage =0;
    Intake intake;
    Chassis drive;
    Shooter shooter;
    Odo odo;
    ElapsedTime timer;
    VoltageSensor voltageSensor;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Hardware.init(hardwareMap);
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
        odo = new Odo();
        intake =new Intake();
        drive =new Chassis(Chassis.State.DRIVE);
        shooter =new Shooter(Shooter.State.ACTIVE);
        shooter.turret.setState(Turret.State.BLUE);
        waitForStart();
        while (opModeIsActive()) {

            currentVoltage = voltageSensor.getVoltage();
            gm1.copy(gamepad1);
            gm2.copy(gamepad2);
            intake.update();
            shooter.update();
            drive.update();
            odo.update();

            if (gm1.right_stick_x>0.65 && prevgm1.right_stick_x<0.65) Odo.offsetX -=40;
            if (gm1.right_stick_x<-0.65 && prevgm1.right_stick_x>-0.65) Odo.offsetX +=40;
            if (gm1.right_stick_y>0.65 && prevgm1.right_stick_y<0.65) Odo.offsetY +=40;
            if (gm1.right_stick_y<-0.65 && prevgm1.right_stick_y>-0.65) Odo.offsetY -=40;

            double heading = -Odo.getHeading() + Math.PI;
            double X = -gm1.left_stick_y;
            double Y = -gm1.left_stick_x;
            double rx = (gm1.right_trigger - gm1.left_trigger);
            double x = X * Math.cos(heading) - Y * Math.sin(heading);
            double y = X * Math.sin(heading) + Y * Math.cos(heading);
            drive.setTargetVector(x, y, rx);

            if (intake.storage.getState() == Storage.State.GOINGTRANSFER){
                gamepad1.rumble(50);
            }
            if (intake.storage.getState() == Storage.State.TRANSFER && intake.latch.getState() == Latch.State.TRANSFER && gm1.crossWasPressed()){
                intake.setState(Intake.State.SHOOT);
                shooter.setState(Shooter.State.SHOOT);
            }
            else if (intake.storage.getState() != Storage.State.TRANSFER && intake.storage.getState()!= Storage.State.SHOOT && gm1.right_bumper) intake.setState(Intake.State.INTAKE);
            else if ((intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER) && gm1.right_bumper) intake.setState(Intake.State.REVERSE);
            else if (intake.storage.getState() != Storage.State.TRANSFER && intake.storage.getState()!= Storage.State.SHOOT && gm1.left_bumper) intake.setState(Intake.State.REVERSE);
            else if (!gm1.right_bumper && !gm1.left_bumper && intake.state != Intake.State.SHOOT) intake.setState(Intake.State.IDLE);
            if (gamepad1.psWasPressed()){
                odo.reset();
            }
            if (gm1.circleWasPressed())intake.storage.byPass();
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);
            telemetry.update();

        }
    }
}