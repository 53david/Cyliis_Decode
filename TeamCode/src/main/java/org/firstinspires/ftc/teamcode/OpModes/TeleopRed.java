package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.Voltage;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class TeleopRed extends LinearOpMode {

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
        isAutonomousActive = false;
        Initializer.start(hardwareMap);
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
        odo = new Odo();
        intake =new Intake();
        drive =new Chassis(Chassis.State.DRIVE);
        shooter =new Shooter(Shooter.State.SHOOT);
        Hood.state = Hood.State.IDLE;
        waitForStart();
        Turret.goalPositionX = 0;
        Turret.goalPositionY = -840;
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

            double heading = -Odo.getHeading() - Math.PI;
            double X = gm1.left_stick_x;
            double Y = -gm1.left_stick_y;
            double rx = (gm1.right_trigger - gm1.left_trigger);
            double x = X * Math.cos(heading) - Y * Math.sin(heading);
            double y = X * Math.sin(heading) + Y * Math.cos(heading);
            drive.setTargetVector(x, y, rx);

            if (Storage.state == Storage.State.GOINGTRANSFER){
                gamepad1.rumble(50);
            }
            if (Storage.state == Storage.State.TRANSFER && gm1.crossWasPressed())Intake.state = Intake.State.SHOOT;
            else if (Storage.state != Storage.State.TRANSFER && Storage.state!= Storage.State.SHOOT && gm1.right_bumper) Intake.state = Intake.State.INTAKE;
            else if ((Storage.state == Storage.State.GOINGTRANSFER || Storage.state == Storage.State.TRANSFER) && gm1.right_bumper) Intake.state = Intake.State.REVERSE;
            else if (Storage.state != Storage.State.TRANSFER && Storage.state!= Storage.State.SHOOT && gm1.left_bumper) Intake.state = Intake.State.REVERSE;
            if (gamepad1.psWasPressed()){
                odo.reset();
            }
            Voltage = 12.0/currentVoltage;
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);
            telemetryM.update();
            telemetry.update();

        }
    }
}