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

@Autonomous(name = "Red Auto NO Duck")
public class RedAutoNoDuck extends LinearOpMode {

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

        drive.setPoseEstimate(new Pose2d(-12, -63, Math.toRadians(90)));

        Trajectory goToAllianceWobble = drive.trajectoryBuilder(new Pose2d(-12, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(-12, -38), Math.toRadians(90))
                .build();
        Trajectory backFromWobble = drive.trajectoryBuilder(goToAllianceWobble.end())
                .back(12)
                .build();
        Trajectory wareHouse = drive.trajectoryBuilder(backFromWobble.end())
                .splineToSplineHeading(new Pose2d(10, -63, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -63, Math.toRadians(180)), Math.toRadians(180))
                .build();
        Trajectory wareHouseDiddleDaddle = drive.trajectoryBuilder(wareHouse.end())
                .splineTo(new Vector2d(55, -46), Math.toRadians(-135))
                .splineTo(new Vector2d(47, -54), Math.toRadians(135))
//                .splineTo(new Vector2d(38, -62), Math.toRadians(180))
//                .splineTo(new Vector2d(61, -57), Math.toRadians(-90))
                .build();

        waitForStart();

        armMotor.setPower(1);

        armMotor.setTargetPosition(-900);

        drive.followTrajectory(goToAllianceWobble);

        intakeMotor.setPower(0.8);
        sleep(3000);
        intakeMotor.setPower(0);

        armMotor.setTargetPosition(-3200);
        intakeMotor.setPower(-1);

        drive.followTrajectory(backFromWobble);
        drive.followTrajectory(wareHouse);

        drive.followTrajectoryAsync(wareHouseDiddleDaddle);
        while(opModeIsActive()) {
            log.add(colorSensor.red() + " " + colorSensor.green());
            if(colorSensor.red() > 1000 && colorSensor.green() > 1000) {
                drive.cancelFollowing();
                break;
            }
        }

        intakeMotor.setPower(0);
        armMotor.setTargetPosition(-900);
    }
}
