package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
@Config
@TeleOp
public class MecanumTest extends LinearOpMode {
    public static double x;
    public static double power;
    DcMotorEx fl,bl,fr,br,motor;
    @Override
    public void runOpMode(){
        Hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        fl= Hardware.mch3;
        fr= Hardware.mch1;
        bl=Hardware.mch2;
        br=Hardware.mch0;
        motor = Hardware.mch3;
        waitForStart();
        while (opModeIsActive()){
            if (x==1){
                motor.setPower(0);
                motor = fl;
            }
            if (x==2){
                motor.setPower(0);
                motor = fr;
            }
            if (x==3){
                motor.setPower(0);
                motor = bl;
            }
            if (x==4){
                motor.setPower(0);
                motor = br;
            }
            motor.setPower(power);
            telemetry.addData("AMPS",motor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
