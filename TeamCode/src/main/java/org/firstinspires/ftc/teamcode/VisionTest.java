package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.FinalWABOTPipeline;

@Autonomous(name = "Vision Test")
public class VisionTest extends LinearOpMode {

    OptimizedRobot robot;
    Telemetry.Log log;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new OptimizedRobot(telemetry, hardwareMap);
        robot.initializeOpenCVPipeline(true, new FinalWABOTPipeline());
        log = telemetry.log();
        waitForStart();
//        log.clear();
        sleep(100);
        log.add(robot.getVisionOutputToken());
        sleep(1000000000);
    }
}