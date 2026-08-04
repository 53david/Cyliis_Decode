package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;

@Autonomous(name="Sper_ca_merge-BLUE")
public class BlueClose extends LinearOpMode {
    public CloseBlue closeBlue;
    @Override
    public void runOpMode() throws InterruptedException{
        closeBlue = new CloseBlue(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            Chassis.state = Chassis.State.PID;
            closeBlue.update();
        }
    }
}
