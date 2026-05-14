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
    Pose2D start = new Pose2D(0,0,Math.PI/2);
    Pose2D finish = new Pose2D(1200,0,Math.PI/2);
    Chassis chassis;
    Odo odo;
    public static boolean isReady;
    public enum State{
        IDLE,
        FORWARD,
        BACKWARD,
    };
    State state = State.IDLE;
    public void updateState(){
        switch (state){
            case IDLE:
                if(!chassis.inPosition(100,100,0.4)) {
                    chassis.setTargetPosition(start);
                }
                if (isReady)
                    state = State.FORWARD;
                break;
            case FORWARD:
                chassis.setTargetPosition(finish);
                if (chassis.inPosition(10,10,0.2))
                    state = State.BACKWARD;
                break;
            case BACKWARD:
                chassis.setTargetPosition(start);
                if (chassis.inPosition(10,10,0.2)) {
                    state = State.IDLE;
                     isReady = false;
                }
                break;
        }
    }
    public void runOpMode(){
        Initializer.start(hardwareMap);
        chassis = new Chassis(Chassis.State.PID);
        odo = new Odo();
        odo.reset();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.ps){
                odo.reset();
            }
            updateState();
            chassis.update();
            odo.update();
            telemetryM.addData("X",Odo.getX());
            telemetryM.addData("Y",Odo.getY());
            telemetryM.addData("Heading",Odo.getHeading());
            telemetryM.addData("True X",Odo.trueX());
            telemetryM.addData("True Y",Odo.trueY());
            telemetryM.addData("True heading",Odo.trueHeading());
            telemetryM.update();
        }
    }

}
