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


@Autonomous(name = "Marker-Crater", group = "Linear Opmode")
public class Marker_Crater extends LinearOpMode {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private ColorSensor sensorColor;
    private GoldAlignDetector detector;
    private BNO055IMU imu;
    private DcMotor hangmotor = null;
    private Servo TMServo = null;

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
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
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

            setDriveSpeed(0, 0, 0, 0);
            sleep(500);

            setDriveSpeed(0.4, 0.4, 0, 0);
            sleep(500);

            while (Math.abs(getHeading() + -10) > HEADING_EPSILON) {
                setDriveSpeed(-0.4, 0.4, -0.4, 0.4);
            }

            setDriveSpeed(0, 0, 0, 0);
            sleep(1000);


            detector.alignPosOffset = -200;

            setDriveSpeed(0, 0, 0, 0);
            sleep(1000);


            if (detector.getAligned()) {

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(1500);


                while (isGray()) {
                    setDriveSpeed(0.375, 0.375, 0, 0);
                    idle();
                }
                //SERVO TEAM MARKER CODE.
                TMServo.setPosition(0);
                setDriveSpeed(0, 0, 0, 0);
                sleep(1000);

                while (Math.abs(getHeading() + 100) > HEADING_EPSILON) {
                    setDriveSpeed(0.5, -0.4, 0.5, -0.4);
                }
                setDriveSpeed(0.35, -0.35, -0.55, 0.5);
                sleep(2100);


                setDriveSpeed(0.5, -0.4, 0, 0);
                sleep(200);


                setDriveSpeed(.5, 0.4, 0, 0);
                sleep(2500);

                setDriveSpeed(.4, 0.5, 0, 0);
                sleep(2500);


                firsttime = 0;
                stop();
                stopMotors();
                idle();
            } else {


                while (Math.abs(getHeading() + 17) > HEADING_EPSILON) {
                    setDriveSpeed(0.4, -0.4, 0.4, -0.4);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(300);


                if (detector.isFound()) {
                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(2000);

                    while (Math.abs(getHeading() + -35) > HEADING_EPSILON) {
                        setDriveSpeed(-0.45, 0.45, -0.45, 0.45);
                    }

                    while (isGray()) {
                        setDriveSpeed(0.375, 0.375, 0, 0);
                    }
                    //SERVO TEAM MARKER CODE
                    TMServo.setPosition(0);

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(250);

                    setDriveSpeed(-0.4,-0.4,0,0);
                    sleep(2000);



                    while (Math.abs(getHeading() + 107) > HEADING_EPSILON) {
                        setDriveSpeed(-0.4, 0.4, -0.4, 0.4);

                    }
                    setDriveSpeed(0.6, -0.6, -0.35, 0.35);
                    sleep(1700);

                    setDriveSpeed(0.45,0.4,0,0);
                    sleep(6000);




                    firsttime = 0;
                    stop();
                    stopMotors();
                    idle();


                } else {
                    while (Math.abs(getHeading() - 40) > HEADING_EPSILON) {
                        setDriveSpeed(-0.4, 0.4, -0.4, 0.4);
                    }

                    setDriveSpeed(0, 0, 0, 0);
                    sleep(1000);

                    setDriveSpeed(0.4, 0.4, 0, 0);
                    sleep(2000);

                    while (Math.abs(getHeading() + 7) > HEADING_EPSILON) {
                        setDriveSpeed(0.45, -0.45, 0.45, -0.45);
                    }

                    while (isGray()) {
                        setDriveSpeed(0.375, 0.375, 0, 0);
                    }
                    TMServo.setPosition(0);
                    setDriveSpeed(0, 0, 0, 0);
                    sleep(1000);



                    while (Math.abs(getHeading() + 103) > HEADING_EPSILON) {
                        setDriveSpeed(0.4, -0.4, 0.4, -0.4);

                    }
                    setDriveSpeed(0.55, -0.55, -0.35, 0.35);
                    sleep(3000);

                    setDriveSpeed(0.45,0.4,0,0);
                    sleep(6000);


                    firsttime = 0;
                    stop();
                    stopMotors();
                    idle();


                }
            }

            stop();
            firsttime = 0;


        }

    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    boolean isGray() {
        final double COLOR_EPSILON = 20;

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

    public void stopMotors() {
        setDriveSpeed(0, 0, 0, 0);
        hangmotor.setPower(0);
    }


    void setDriveSpeed(double left, double right, double bleft, double bright) {
        front_left.setPower(left);
//        leftDrive2.setPower(left * 0.9);
        front_right.setPower(right);
//        rightDrive2.setPower(right * 0.9);
        back_left.setPower(bleft);

        back_right.setPower(bright);

    }


}


