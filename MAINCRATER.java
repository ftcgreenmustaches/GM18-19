package org.firstinspires.ftc.teamcode;

import android.media.Image;
import android.util.Log;

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
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRColor;

import java.util.IllegalFormatCodePointException;

@Autonomous(name = "MAINCRATER", group = "Linear Opmode")
public class MAINCRATER extends LinearOpMode {

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private ColorSensor sensorColor;
    private GoldAlignDetector detector;
    private BNO055IMU imu;
    private DcMotor hangmotor = null;
    private Servo TMServo = null;
    private Servo PlaneServo = null;
    private ColorSensor linesensor = null;

    final double HEADING_EPSILON = 1.5;// Original=1.5


    double calcRotationError(double target) {
        double err = target - getHeading();
        double revError;
        if (err < 0) {
            revError = err - 360;
        } else {
            revError = err + 360;
        }

        if (Math.abs(revError) < Math.abs(err)) {
            err = revError;
        }

        return err;
    }

    void rotateToAngle(double target) {
        target = -target;
        double error = calcRotationError(target);
        while (Math.abs(error) > HEADING_EPSILON && !isStopRequested()) {
            double pwr = Math.abs(error) * 0.02;
            pwr = Math.min(pwr, 0.55);
            pwr = Math.max(pwr, 0.45);
            if (error < 0) pwr = -pwr;

            setDriveSpeed(-pwr, pwr, -pwr, pwr);
            error = calcRotationError(target);
            Log.d("GRYO", "" + System.currentTimeMillis() + "," + getHeading() + "," + pwr + "," + error + "\n");

            telemetry.addData("Gyro Value:", getHeading());
            telemetry.addData("Power:", pwr);
            telemetry.update();
        }
    }

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
        PlaneServo = hardwareMap.get(Servo.class, "planeservo");
        linesensor = hardwareMap.get(ColorSensor.class, "linesensor");

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
        detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -175; // How far from center frame to offset this alignment zone.
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

            idle();

            final double HEADING_EPSILON = 1.5;// Original=1.5

            int MineralPosition;

            setDriveSpeed(0, 0, 0, 0);
            sleep(1000);

            if (detector.getAligned()) {
                telemetry.addData("Position:", "Center");
                telemetry.update();
                MineralPosition = 1;
            } else if (detector.isFound()) {
                telemetry.addData("Position:", "Right");
                telemetry.update();
                MineralPosition = 2;
            } else {
                telemetry.addData("Position:", "Left");
                telemetry.update();
                MineralPosition = 3;
            }

            hangmotor.setPower(1);
            for (int i = 0; i < 32 && !isStopRequested(); ++i) {
                sleep(100);
                idle();
            }

            hangmotor.setPower(0);

            setDriveSpeed(0, 0, 0, 0);
            sleep(1000);


            front_left.setPower(0.6);
            front_right.setPower(-0.6);
            back_left.setPower(-(0.75) * 0.6);
            back_right.setPower((0.75) * 0.6);
            sleep(400);


            if (MineralPosition == 1) {

/* new code added */
                setDriveSpeed(0.4, 4, 0, 0);
                sleep(400);

                rotateToAngle(3);

                setDriveSpeed(0, 0, 0, 0);
                sleep(1000);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(1700);

               while (isGraylinesensor()) {
                    setDriveSpeed(-0.4, -0.4, 0, 0);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);

                setDriveSpeed(0.3, 0.3, 0, 0);
                sleep(1500);
                /*new code ended */
                rotateToAngle(-85);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(3150);

                rotateToAngle(-140);

                setDriveSpeed(0.3, 0.3, 0, 0);
                sleep(1000);

                while (isGray()) {
                    setDriveSpeed(0.35, 0.35, 0, 0);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(0);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(1);

                setDriveSpeed(-0.55, -0.4, 0, 0);
                sleep(2000);

                PlaneServo.setPosition(0.5);


                setDriveSpeed(0, 0, 0, 0);
                sleep(500);


                setDriveSpeed(-0.6, -0.5, -0.6, -0.5);
                sleep(250);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);

                firsttime = 0;
                stopMotors();
                stop();

            } else if (MineralPosition == 2) {
                setDriveSpeed(0.4, 4, 0, 0);
                sleep(400);

                rotateToAngle(35);

                setDriveSpeed(0, 0, 0, 0);
                sleep(300);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(1900);

                setDriveSpeed(-.4,-.4,0,0);
                sleep(1000);

                while (isGraylinesensor()) {
                    setDriveSpeed(-0.4, -0.4, 0, 0);
                }

                rotateToAngle(0);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);

                setDriveSpeed(0.3, 0.3, 0, 0);
                sleep(1500);


                rotateToAngle(-75);


                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(2950);

                rotateToAngle(-135);


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

                PlaneServo.setPosition(0.5);


                setDriveSpeed(0, 0, 0, 0);
                sleep(500);


                setDriveSpeed(-0.3, -0.3, -0.3, -0.3);
                sleep(1500);

                firsttime = 0;
                stop();
                stopMotors();
                idle();


            } else {
                setDriveSpeed(0.4, 4, 0, 0);
                sleep(400);

                rotateToAngle(-15);

                setDriveSpeed(0, 0, 0, 0);
                sleep(300);

                setDriveSpeed(0.4, 0.4, 0, 0);
                sleep(2000);

                while (isGraylinesensor()) {
                    setDriveSpeed(-0.4, -0.4, 0, 0);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(250);

                setDriveSpeed(0.3, 0.3, 0, 0);
                sleep(1500);

                 rotateToAngle(-85);

                setDriveSpeed(0.5, 0.5, 0, 0);
                sleep(2400);

                rotateToAngle(-135);


                while (isGray()) {
                    setDriveSpeed(0.4, 0.4, 0, 0);
                }

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(0);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);
                TMServo.setPosition(1);

                setDriveSpeed(-0.7, -0.7, 0, 0);
                PlaneServo.setPosition(0.5);
                sleep(2000);

                setDriveSpeed(0, 0, 0, 0);
                sleep(500);


                firsttime = 0;
                stop();
                stopMotors();
                idle();


            }


            stopMotors();
            stop();
            firsttime = 0;


        }

    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
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


