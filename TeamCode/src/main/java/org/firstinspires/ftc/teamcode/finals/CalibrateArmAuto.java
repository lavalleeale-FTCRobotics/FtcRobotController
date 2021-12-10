package org.firstinspires.ftc.teamcode.finals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Calibrate Arm", group = "Calibration")
public class CalibrateArmAuto extends LinearOpMode {

    DcMotor arm;

    OptimizedRobot robot;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);

        log = telemetry.log();

        robot.initializeDriveMotors(true);

        arm = robot.getMotor("arm", DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        arm.setPower(1);

        sleep(1000);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}