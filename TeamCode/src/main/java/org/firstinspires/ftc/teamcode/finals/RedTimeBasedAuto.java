package org.firstinspires.ftc.teamcode.finals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedDriveFunctions;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.internal.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Time Based Auto (Red)")
public class RedTimeBasedAuto extends LinearOpMode {

    DcMotor duckSpinner;

    OptimizedRobot robot;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);

        log = telemetry.log();

        duckSpinner = robot.getMotor("duckSpinner");

        robot.initializeDriveMotors(true);


        waitForStart();

        robot.getDriveFunctions().linearDrive(-0.6f);

        sleep(100);

        robot.getDriveFunctions().stopMotors();

        duckSpinner.setPower(RobotConfig.SPINNER_SPEED);

        sleep(4000);

        robot.getDriveFunctions().strafeLinear(OptimizedDriveFunctions.Direction.LEFT, 0.9);

        sleep(350);

        duckSpinner.setPower(0);

        robot.getDriveFunctions().linearDrive(0.4f);

        sleep(5500);

        robot.getDriveFunctions().stopMotors();
    }
}