package org.firstinspires.ftc.teamcode.finals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Simple Auto")
public class SimpleAuto extends LinearOpMode {

    DcMotor arm;

    CRServo leftServo, rightServo;

    OptimizedRobot robot;

    Telemetry.Log log;

    int targetPosition = -900;
    int startingPosition = -128;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);

        log = telemetry.log();

        robot.initializeDriveMotors(true);

        arm = robot.getMotor("arm", DcMotor.RunMode.RUN_TO_POSITION);

        leftServo = hardwareMap.crservo.get("Left Servo");
        rightServo = hardwareMap.crservo.get("Right Servo");

        waitForStart();

        arm.setPower(1d);

        arm.setTargetPosition(startingPosition);
        sleep(100);

        leftServo.setPower(-0.5);
        rightServo.setPower(0.5);

        sleep(1000);

        arm.setTargetPosition(targetPosition);

        sleep(100);

        robot.getDriveFunctions().linearDrive(0.9f);

        sleep(1000);

        robot.getDriveFunctions().stopMotors();
    }
}