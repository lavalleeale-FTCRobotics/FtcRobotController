package org.firstinspires.ftc.teamcode.finals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Auto for Scarab match")
public class AutoWithScarabs extends LinearOpMode {

    DcMotor duckSpinner, armMotor, intakeMotor, odometryMotor;
    OptimizedRobot robot;
    ColorSensor colorSensor;

    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {

        log = telemetry.log();

        robot = new OptimizedRobot(telemetry, hardwareMap);

        duckSpinner = robot.getMotor("duckSpinner");
        armMotor = robot.getMotor("arm", DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor = robot.getMotor("intake");
        odometryMotor = robot.getMotor("odometry");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        colorSensor.enableLed(true);

        org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-40.5, 63, Math.toRadians(-90)));

        Trajectory forwardForDuck = drive.trajectoryBuilder(new Pose2d(-40.5, 63, Math.toRadians(-90)))
                .splineTo(new Vector2d(-55.5, 49), Math.toRadians(-90))
                .build();
        Trajectory reverseForDuck = drive.trajectoryBuilder(forwardForDuck.end())
                .back(11)
                .build();
        Trajectory goToAllianceWobbleFromDuck = drive.trajectoryBuilder(reverseForDuck.end())
                .splineTo(new Vector2d(-30, 24), Math.toRadians(0))
                .build();
        Trajectory backFromWobble = drive.trajectoryBuilder(goToAllianceWobbleFromDuck.end())
                .back(12)
                .build();
        Trajectory toParkZone = drive.trajectoryBuilder(new Pose2d(-42, 24, Math.toRadians(180)))
                .splineTo(new Vector2d(-60, 35), Math.toRadians(180))
                .build();

        waitForStart();

        armMotor.setPower(1);

        drive.followTrajectory(forwardForDuck);

        drive.followTrajectory(reverseForDuck);

        duckSpinner.setPower(0.6);

        sleep(2500);

        duckSpinner.setPower(0);
        armMotor.setTargetPosition(-900);

        drive.followTrajectory(goToAllianceWobbleFromDuck);

        intakeMotor.setPower(0.8);
        sleep(3000);
        intakeMotor.setPower(0);

        armMotor.setTargetPosition(-3200);
        intakeMotor.setPower(-1);

        drive.followTrajectory(backFromWobble);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(toParkZone);

        intakeMotor.setPower(0);
        armMotor.setTargetPosition(-900);
    }
}