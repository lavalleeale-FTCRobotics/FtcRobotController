package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class TestRoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0)).back(30).build();
//        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-30, 0, 0)).strafeLeft(30).build();
//        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-30, 30, 0)).forward(30).build();
//        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(0, 30, 0)).strafeRight(30).build();

        while(!isStarted()){
            telemetry.log().add(String.valueOf(drive.imu.getAngularOrientation().firstAngle));
            telemetry.log().add(String.valueOf(drive.getRawExternalHeading()));
        }

        if(isStopRequested()) return;

//        drive.turn(Math.toRadians(180));

        sleep(1000);

//        while(true){
//            drive.followTrajectory(traj1);
//            sleep(100);
//            drive.followTrajectory(traj2);
//            sleep(100);
//            drive.followTrajectory(traj3);
//            sleep(100);
//            drive.followTrajectory(traj4);
//            sleep(100);
//        }
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 0), Math.toRadians(90)).build());

        while(true){
            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(30, 0, Math.toRadians(90)))
                    .splineTo(new Vector2d(0, 30), Math.toRadians(180))
                    .splineTo(new Vector2d(-30, 0), Math.toRadians(-90))
                    .splineTo(new Vector2d(0, -30), Math.toRadians(0))
                    .splineTo(new Vector2d(30, 0), Math.toRadians(90))
                    .build());
        }
    }
}
