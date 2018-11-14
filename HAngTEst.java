package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;

import java.util.IllegalFormatCodePointException;

/**
 * Created by aparikh1 on 11/1/2017.
 */

@Autonomous(name = "HangTEst", group = "Linear Opmode")
public class HAngTEst extends LinearOpMode {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private ColorSensor sensorColor;
    private GoldAlignDetector detector;
    private BNO055IMU imu;
    private DcMotor hangmotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");


        telemetry.update();
        front_left = hardwareMap.get(DcMotor.class, "motorleft");
        front_right = hardwareMap.get(DcMotor.class, "motorright");
        back_left = hardwareMap.get(DcMotor.class, "backmotorleft");
        back_right = hardwareMap.get(DcMotor.class, "backmotorright");
        sensorColor = hardwareMap.get(ColorSensor.class, "colorsensor");
        hangmotor = hardwareMap.get(DcMotor.class, "hangmotor");


        front_left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right.setDirection(DcMotor.Direction.FORWARD);


        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        // Optional Tuning
        detector.alignSize = 180; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();


        waitForStart();

        int firsttime = 1;
        while (opModeIsActive() && firsttime == 1) {
            hangmotor.setPower(1);
            sleep(3650);

            hangmotor.setPower(0);
            /* STRAFING */
            front_left.setPower(0.45);
            front_right.setPower(-0.45);
            back_left.setPower(-0.45);
            back_right.setPower(0.45);
            sleep(250);
            /* TURN*/
            front_left.setPower(0.45);
            front_right.setPower(-0.45);
            back_left.setPower(0.45);
            back_right.setPower(-0.45);
            sleep(100);
            /*STRAFFING*/
            front_left.setPower(0.45);
            front_right.setPower(-0.45);
            back_left.setPower(-0.45);
            back_right.setPower(0.45);
            sleep(100);






            firsttime=0;











        }



        firsttime = 0;
    }


}


