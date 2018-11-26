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

@Autonomous(name = "Crater-Regular", group = "Linear Opmode")
public class Crater_Regular extends LinearOpMode {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private ColorSensor sensorColor;
    private GoldAlignDetector detector;
    private BNO055IMU imu;
    private DcMotor hangmotor = null;
    private Servo TMServo = null;
    private DcMotor mineralmotor = null;
    private ColorSensor linesensor = null;
    private Servo PlaneServo = null;

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
        TMServo = hardwareMap.get(Servo.class, "TMServo");
        mineralmotor = hardwareMap.get(DcMotor.class, "mineralmotorleft");
        linesensor = hardwareMap.get(ColorSensor.class, "linesensor");
        PlaneServo = hardwareMap.get(Servo.class, "planeservo");

        front_left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        back_right.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        // Optional Tuning
        detector.alignSize = 180; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -100; // How far from center frame to offset this alignment zone.
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


            final double HEADING_EPSILON = 1.5;// Original=1.5

            hangmotor.setPower(1);
            sleep(3700);

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

            setDriveSpeed(0, 0, 0, 0);
            sleep(500);

            setDriveSpeed(0.4, 0.4, 0, 0);
            sleep(500);

            while (Math.abs(getHeading() + -16) > HEADING_EPSILON) {
                setDriveSpeed(-0.4, 0.4, -0.4, 0.4);
            }

            setDriveSpeed(0, 0, 0, 0);
            sleep(300);

            detector.alignPosOffset = -150;

            setDriveSpeed(0, 0, 0, 0);
            sleep(500);

            if (detector.getAligned()) {


                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(1050);

                while (isGraylinesensor()) {
                    setDriveSpeed(-0.25, -0.25, 0, 0);
                }
                setDriveSpeed(0, 0, 0, 0);
                sleep(500);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(699);


                while (Math.abs(getHeading() - 100) > HEADING_EPSILON) {
                    setDriveSpeed(-0.375, 0.375, -0.375, 0.375);
                }

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(2800);

                while (Math.abs(getHeading() - 145) > HEADING_EPSILON) {
                    setDriveSpeed(-0.375, 0.375, -0.375, 0.375);
                }

                setDriveSpeed(0.3, 0.3, 0, 0);
                sleep(1000);

                while (isGray()) {
                    setDriveSpeed(0.4, 0.4, 0, 0);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(0);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(1);

                setDriveSpeed(-0.4, -0.4, 0, 0);
                sleep(2000);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);


                setDriveSpeed(-0.3, -0.3, -0.3, -0.3);
                sleep(2000);

                PlaneServo.setPosition(0.5);
                setDriveSpeed(0, 0, 0, 0);
                sleep(500);


                stop();
                firsttime = 0;

            } else {


                while (Math.abs(getHeading() + 13) > HEADING_EPSILON) {
                    setDriveSpeed(0.35, -0.35, 0.35, -0.35);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(1000);

                if (detector.getAligned()) {
                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(2000);

                    while (isGraylinesensor()) {
                        setDriveSpeed(-0.3, -0.3, 0, 0);
                    }


                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);

                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(699);


                    while (Math.abs(getHeading() - 83) > HEADING_EPSILON) {
                        setDriveSpeed(-0.4, 0.4, -0.4, 0.4);
                    }


                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(3300);

                    while (Math.abs(getHeading() - 143) > HEADING_EPSILON) {
                        setDriveSpeed(-0.45, 0.45, -0.45, 0.45);
                    }

                    while (isGray()) {
                        setDriveSpeed(0.4, 0.4, 0, 0);
                    }


                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);
                    TMServo.setPosition(0);

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);
                    TMServo.setPosition(1);

                    setDriveSpeed(-0.4, -0.4, 0, 0);
                    sleep(2000);

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);


                    setDriveSpeed(-0.3, -0.3, -0.3, -0.3);
                    sleep(1000);

                    PlaneServo.setPosition(0.5);

                    stop();
                    firsttime = 0;


                } else {
                    while (Math.abs(getHeading() - 30) > HEADING_EPSILON) {
                        setDriveSpeed(-0.45, 0.45, -0.45, 0.45);
                    }

                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(2000);

                    while (isGraylinesensor()) {
                        setDriveSpeed(-0.25, -0.25, 0, 0);

                    }

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(250);

                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(699);


                    while (Math.abs(getHeading() - 90) > HEADING_EPSILON) {
                        setDriveSpeed(-0.45, 0.45, -0.45, 0.45);
                    }
                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(2250);

                    while (Math.abs(getHeading() - 150) > HEADING_EPSILON) {
                        setDriveSpeed(-0.375, 0.375, -0.375, 0.375);
                    }

                    while (isGray()) {
                        setDriveSpeed(0.4, 0.4, 0, 0);
                    }

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);
                    TMServo.setPosition(0);

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);
                    TMServo.setPosition(1);

                    setDriveSpeed(-0.4, -0.4, 0, 0);
                    sleep(2000);

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(500);


                    setDriveSpeed(-0.3, -0.3, -0.3, -0.3);
                    sleep(1000);

                    PlaneServo.setPosition(0.5);

                    stop();
                    firsttime = 0;

                }
            }


        }

        firsttime = 0;
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    void setDriveSpeed(double left, double right, double bleft, double bright) {
        front_left.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        front_right.setPower(right);
//        rightDrive2.setPower(right * 0.9);
        back_left.setPower(bleft);

        back_right.setPower(bright);

    }

    boolean isGray() {
        final double COLOR_EPSILON = 30;

        if (Math.abs(sensorColor.red() - sensorColor.blue()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(sensorColor.red() - sensorColor.green()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(sensorColor.blue() - sensorColor.green()) > COLOR_EPSILON) {
            return false;
        }

        // must be gray
        return true;
    }

    boolean isGraylinesensor() {
        final double COLOR_EPSILON = 4;

        if (Math.abs(linesensor.red() - linesensor.blue()) >= COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(linesensor.red() - linesensor.green()) >= COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(linesensor.blue() - linesensor.green()) >= COLOR_EPSILON) {
            return false;
        }

        // must be gray
        return true;
    }


}


