package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.examples.SampleControllerMapping;
import org.firstinspires.ftc.teamcode.examples.SampleHardwareAliasMapping;

import java.util.List;

@TeleOp(name = "TensorFlow Object Detection Test")
public class CameraTeleOpTest extends LinearOpMode {

    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BRMotor;
    DcMotor BLMotor;

    OptimizedRobot robot;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static String VUFORIA_KEY = "AS2jf2L/////AAABmXgZ8WffLklTjSGx4BHLMEyIUzz/MDvofYi6H+Wn9qY4aJN5vPfPNmDqPOZkFax2dve7+6w6JCk+CoHc0hAq8UueKVMgQ4ma5xOB0mrS8RdkV330C2uom99XO9gX6Pbvy/CZHU3Is0BXnZJZ0ZhG8vqrJ+c0KFl1uiFpLLE5KeT82u4FA9cO5ZSR9iWyVGHkFYlXpd/nuKdWVf25KGhElSSBDIhBTQV6ykNZqySS8sqdWg+aG7uqdbEYV40ac+iL4Fm6OyyotS8CwdY9TvYsemRlMHsK1OY3ngIWZ9rpQ9dx/oPDDtezc0WhpfpBsoxtta78pJNLRofHhulw3sIAGgN5bwAwNrWUzMzEYhw15Y7R";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new OptimizedRobot(gamepad1, gamepad2, telemetry, hardwareMap, new SampleControllerMapping(), new SampleHardwareAliasMapping());

        FRMotor = robot.getMotor("FRMotor", DcMotorSimple.Direction.REVERSE);
        FLMotor = robot.getMotor("FLMotor", DcMotorSimple.Direction.FORWARD);
        BLMotor = robot.getMotor("BLMotor", DcMotorSimple.Direction.FORWARD);
        BRMotor = robot.getMotor("BRMotor", DcMotorSimple.Direction.REVERSE);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        FRMotor.setPower(0.3);
        BRMotor.setPower(-0.3);
        FLMotor.setPower(0.3);
        BLMotor.setPower(-0.3);

        boolean objectDetected = false;

        waitForStart();

        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.update();
                objectDetected = true;
            }
            if (objectDetected)
                break;
        }

        FRMotor.setPower(0.7);
        BRMotor.setPower(0.7);
        FLMotor.setPower(0.7);
        BLMotor.setPower(0.7);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
