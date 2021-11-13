package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedDriveFunctions;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;

@Autonomous
public class TimeBasedAuto extends LinearOpMode {

    DcMotor duckSpinner;

    OptimizedRobot robot;
    OptimizedController controller1;
    OptimizedController controller2;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap, new SampleControllerMapping());
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot.initializeDriveMotors(true);

        duckSpinner = robot.getMotor("duckSpinner");

        waitForStart();

        duckSpinner.setPower(-0.6);

        sleep(1500);

        duckSpinner.setPower(0);

        robot.getDriveFunctions().linearDrive(0.9f);

        sleep(350);

        robot.getDriveFunctions().turn(OptimizedDriveFunctions.Direction.LEFT, 0.7);

        sleep(420);

        robot.getDriveFunctions().linearDrive(0.9f);

        sleep(2000);

        robot.getDriveFunctions().stopMotors();
    }
}