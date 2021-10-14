package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test TeleOP")
public class ExampleTeleOp extends OpMode {
    DcMotor FLMotor;
    DcMotor FRMotor;
    DcMotor BLMotor;
    DcMotor BRMotor;
    DcMotor duckSpinner;
    Telemetry.Log log;

    public void init() {
        FLMotor = hardwareMap.dcMotor.get("FLMotor");
        FRMotor = hardwareMap.dcMotor.get("FRMotor");
        BLMotor = hardwareMap.dcMotor.get("BLMotor");
        BRMotor = hardwareMap.dcMotor.get("BRMotor");
        duckSpinner = hardwareMap.dcMotor.get("Duck spinner");
        log = telemetry.log();

        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
//        double drive = gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double twist = gamepad1.right_stick_x;
//        double[] speeds = {
//                (drive + strafe + twist),
//                (drive - strafe - twist),
//                (drive - strafe + twist),
//                (drive + strafe - twist)
//        };
//        double max = Math.abs(speeds[0]);
//        for (int i = 1; i < speeds.length; i++) {
//            max = Math.max(max, Math.abs(speeds[i]));
//        }
//        if (max > 1) {
//            for (int i = 0; i < speeds.length; i++) {
//                speeds[i] /= max;
//            }
//        }
//
//
//        FLMotor.setPower(speeds[0]);
//        FRMotor.setPower(speeds[1]);
//        BLMotor.setPower(speeds[2]);
//        BRMotor.setPower(speeds[3]);

        FRMotor.setPower(gamepad1.right_stick_y * 0.9);
        BRMotor.setPower(gamepad1.right_stick_y * 0.9);
        FLMotor.setPower(gamepad1.left_stick_y * 0.9);
        BLMotor.setPower(gamepad1.left_stick_y * 0.9);

        duckSpinner.setPower(-gamepad1.right_trigger);
        log.add(String.valueOf(-gamepad1.right_trigger));
    }
}
