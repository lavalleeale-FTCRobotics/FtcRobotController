package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.internal.OptimizedDriveFunctions;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Example Auto")
@Disabled
public class ExampleAuto extends LinearOpMode {

    OptimizedRobot robot;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);

        log = telemetry.log();

        robot.initializeDriveMotors(true);

        waitForStart();

        // Move forwards with full power
        robot.getDriveFunctions().linearDrive(1);

        sleep(200);

        // Strafe left with full power
        robot.getDriveFunctions().strafeLinear(OptimizedDriveFunctions.Direction.LEFT, 1);

        sleep(200);

        // Move backwards with full power
        robot.getDriveFunctions().linearDrive(-1);

        sleep(200);

        // Strafe right with full power
        robot.getDriveFunctions().strafeLinear(OptimizedDriveFunctions.Direction.RIGHT, 1);
    }
}