package org.firstinspires.ftc.teamcode.Wrappers;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;


public class Initializer {
    public static double Voltage = 0;
    public static boolean isAutonomousActive = false;
    public static TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public static FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    public static CRServo intakeMotor;
    public static BHI260IMU imu;
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public static CRServo shoot1;
    public static CRServo shoot2;
    public static CRServo spin;
    public static ServoImplEx servo1;
    public static ServoImplEx servo2;
    public static ServoImplEx transfer;
    public static ServoImplEx hood;
    public  static org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver pp;
    public static AnalogInput encoder;
    public static ColorRangeSensor color;
    public static DigitalChannel proximitySensor;
    public static Gamepad prevgm1,prevgm2;
    public static Gamepad gm1,gm2;
    public static WebcamName webcam;
    public static Limelight3A limelight3A;
    public static void start(HardwareMap hwMap){
        prevgm1 = new Gamepad();
        prevgm2 = new Gamepad();
        gm1 = new Gamepad();
        gm2 = new Gamepad();
        pp = hwMap.get(org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.class,"odo");
        imu = hwMap.get(BHI260IMU.class,"imu");
        intakeMotor = hwMap.get(CRServo.class,"sch0");
        transfer = hwMap.get(ServoImplEx.class,"ssh0");
        frontLeft = hwMap.get(DcMotorEx.class,"mch3");
        frontRight = hwMap.get(DcMotorEx.class,"mch1");
        backRight = hwMap.get(DcMotorEx.class,"mch0");
        backLeft = hwMap.get(DcMotorEx.class,"mch2");
        shoot1 = hwMap.get(CRServo.class,"sch1");
        shoot2 = hwMap.get(CRServo.class,"sch2");
        spin = hwMap.get(CRServo.class,"sch3");
        servo1 = hwMap.get(ServoImplEx.class,"ssh1");
        servo2 = hwMap.get(ServoImplEx.class,"ssh2");
        encoder = hwMap.get(AnalogInput.class,"spindexer");
        hood = hwMap.get(ServoImplEx.class,"ssh5");
        proximitySensor =hwMap.get(DigitalChannel.class,"bb");

    }
}