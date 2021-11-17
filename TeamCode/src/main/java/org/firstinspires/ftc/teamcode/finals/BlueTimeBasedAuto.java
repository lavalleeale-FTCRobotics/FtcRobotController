package org.firstinspires.ftc.teamcode.finals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedDriveFunctions;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.internal.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Time Based Auto (Blue)")
public class BlueTimeBasedAuto extends LinearOpMode {

    DcMotor duckSpinner;

    OptimizedRobot robot;
    OptimizedController controller1;
    OptimizedController controller2;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot.initializeDriveMotors(true);

        duckSpinner = robot.getMotor("duckSpinner");

        waitForStart();

        robot.getDriveFunctions().linearDrive(-0.6f);

        sleep(100);

        robot.getDriveFunctions().stopMotors();

        duckSpinner.setPower(-RobotConfig.SPINNER_SPEED);

        sleep(4000);

        robot.getDriveFunctions().linearDrive(0.9f);

        sleep(350);

        duckSpinner.setPower(0);

        robot.getDriveFunctions().turn(OptimizedDriveFunctions.Direction.LEFT, 0.7);

        sleep(350);

        robot.getDriveFunctions().linearDrive(0.4f);

        sleep(5500);

        robot.getDriveFunctions().stopMotors();
    }
}