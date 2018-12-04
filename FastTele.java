package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

/**
 * This file provides basic Telop driving for a simple mecanum drive robot.
 * This OpMode uses the T_DrivetrainOnlyHardware hardware class to define the devices on the robot.
 * All device access is managed through the T_DrivetrainOnlyHardware class.
 * This particular OpMode executes a code to run the mecanum wheels to allow for them to work correctly
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "FastTele", group = "FTCRed")
public class FastTele extends OpMode {
    /* Declare OpMode members. */
    public DcMotor mineralmotorleft = null;
    public DcMotor hangmotor = null;
    public DcMotor flipmotor = null;
    public ColorSensor colorSensor = null;
    public DcMotor spinner = null;
    public Servo TMServo = null;
    private Servo PlaneServo = null;
    private ColorSensor linesensor = null;
    boolean backuparm = false;
    int lastPos;

    Hardware robot = new Hardware(); // use the class created to define a robot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello FTC Red Driver");    //
        mineralmotorleft = hardwareMap.get(DcMotor.class, "mineralmotorleft");
        hangmotor = hardwareMap.get(DcMotor.class, "hangmotor");
        flipmotor = hardwareMap.get(DcMotor.class, "flipmotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        TMServo = hardwareMap.get(Servo.class, "TMServo");
        PlaneServo = hardwareMap.get(Servo.class, "planeservo");
        linesensor = hardwareMap.get(ColorSensor.class, "linesensor");

        mineralmotorleft.setDirection(DcMotor.Direction.REVERSE);
        backuparm = false;

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if (gamepad2.left_stick_button) {
            backuparm = true;

        } else if (gamepad2.right_stick_button) {
            backuparm = false;
        }


        if (!backuparm) {
            double rightX_G1;
            double rightY_G1;
            double leftX_G1;
            double leftY_G1;
            double leftX_G2;
            double leftY_G2;
            double rightX_G2;
            double rightY_G2;
            rightX_G1 = gamepad1.right_stick_x;
            rightY_G1 = -gamepad1.right_stick_y;

            leftX_G1 = -gamepad1.left_stick_x;
            leftY_G1 = -gamepad1.left_stick_y;
            leftX_G2 = gamepad2.left_stick_x;
            leftY_G2 = gamepad2.left_stick_y;
            rightX_G2 = gamepad2.right_stick_x;
            rightY_G2 = gamepad2.right_stick_y;

            robot.frontLeftMotor.setPower((leftY_G1 + rightX_G1 + leftX_G1) * 0.6);
            robot.rearLeftMotor.setPower((leftY_G1 + rightX_G1 - leftX_G1) * 0.6);
            robot.rearRightMotor.setPower((leftY_G1 - rightX_G1 + leftX_G1) * 0.6);
            robot.frontRightMotor.setPower((leftY_G1 - rightX_G1 - leftX_G1) * 0.6);


            if (gamepad1.dpad_up) {
                hangmotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                hangmotor.setPower(-1);
            } else {
                hangmotor.setPower(0);
            }

            double armPos = mineralmotorleft.getCurrentPosition();

            //telemetry.addData("encoder-fdw", armPos + "busy=" + mineralmotorleft.isBusy());
            //telemetry.update();


            if (gamepad2.a) {

                if (armPos > 1250) {
                    mineralmotorleft.setTargetPosition(1200);
                    mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mineralmotorleft.setPower(0.25);
                } else if (armPos > 550) {
                    mineralmotorleft.setTargetPosition(500);
                    mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mineralmotorleft.setPower(0.25);
                } else {
                    //    telemetry.addData("Braking", mineralmotorleft.getCurrentPosition());
                    mineralmotorleft.setPower(0.02);
                    mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }


            } else if (gamepad2.y) {
                mineralmotorleft.setTargetPosition(1600);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mineralmotorleft.setPower(0.5);


            } else if (gamepad2.b) {
                mineralmotorleft.setTargetPosition(300);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mineralmotorleft.setPower(0.4);

            } else if (mineralmotorleft.getCurrentPosition() < 1250 && mineralmotorleft.getCurrentPosition() > 200) {
                //  telemetry.addData("Braking", mineralmotorleft.getCurrentPosition());
                mineralmotorleft.setPower(0.03);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                //    telemetry.addData("Braking", "not");
                mineralmotorleft.setPower(0);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            if (gamepad2.left_bumper) {
                spinner.setPower(-.69);
            } else if (gamepad2.right_bumper) {
                spinner.setPower(0.69);
            } else {
                spinner.setPower(0);
            }


            if (gamepad1.a) {
                TMServo.setPosition(0.6);
                PlaneServo.setPosition(0.9);
            }


            if (gamepad1.y) {
                telemetry.addData("Red:1:", colorSensor.red());
                telemetry.addData("Green:1:", colorSensor.green());
                telemetry.addData("Blue:1:", colorSensor.blue());
                telemetry.addData("Red2:", linesensor.red());
                telemetry.addData("Green2:", linesensor.green());
                telemetry.addData("Blue2:", linesensor.blue());
                telemetry.addData("IsGray", isGray());

            }


            if (gamepad1.b) {
                flipmotor.setPower(1);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mineralmotorleft.setPower(0.35);

            } else {
                flipmotor.setPower(0);
            }
            if (gamepad2.back) {
                mineralmotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        } else {//must be backuparm

            mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double rightX_G1;
            double rightY_G1;
            double leftX_G1;
            double leftY_G1;
            double leftX_G2;
            double leftY_G2;
            double rightX_G2;
            double rightY_G2;
            rightX_G1 = gamepad1.right_stick_x;
            rightY_G1 = -gamepad1.right_stick_y;

            leftX_G1 = -gamepad1.left_stick_x;
            leftY_G1 = -gamepad1.left_stick_y;
            leftX_G2 = gamepad2.left_stick_x;
            leftY_G2 = gamepad2.left_stick_y;
            rightX_G2 = gamepad2.right_stick_x;
            rightY_G2 = gamepad2.right_stick_y;

            robot.frontLeftMotor.setPower((leftY_G1 + rightX_G1 + leftX_G1) * 0.6);
            robot.rearLeftMotor.setPower((leftY_G1 + rightX_G1 - leftX_G1) * 0.6);
            robot.rearRightMotor.setPower((leftY_G1 - rightX_G1 + leftX_G1) * 0.6);
            robot.frontRightMotor.setPower((leftY_G1 - rightX_G1 - leftX_G1) * 0.6);


            if (gamepad1.dpad_up) {
                hangmotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                hangmotor.setPower(-1);
            } else {
                hangmotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                spinner.setPower(-.69);
            } else if (gamepad2.right_bumper) {
                spinner.setPower(0.69);
            } else {
                spinner.setPower(0);
            }


            if (gamepad1.a) {
                TMServo.setPosition(0.6);
                PlaneServo.setPosition(0.9);
            }


            if (gamepad1.y) {
                telemetry.addData("Red:1:", colorSensor.red());
                telemetry.addData("Green:1:", colorSensor.green());
                telemetry.addData("Blue:1:", colorSensor.blue());
                telemetry.addData("Red2:", linesensor.red());
                telemetry.addData("Green2:", linesensor.green());
                telemetry.addData("Blue2:", linesensor.blue());
                telemetry.addData("IsGray", isGray());

            }


            if (gamepad1.b) {
                flipmotor.setPower(1);
                mineralmotorleft.setPower(0.35);

            } else {
                flipmotor.setPower(0);
            }
            if (gamepad2.y){
                mineralmotorleft.setPower(0.7);
            }else if(gamepad2.a){
                mineralmotorleft.setPower(-0.2);
            }else {
                mineralmotorleft.setPower(0);
            }





        }


    }

    boolean isGray() {
        final double COLOR_EPSILON = 30;

        if (Math.abs(colorSensor.red() - colorSensor.blue()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(colorSensor.red() - colorSensor.green()) > COLOR_EPSILON) {
            return false;
        }

        if (Math.abs(colorSensor.blue() - colorSensor.green()) > COLOR_EPSILON) {
            return false;
        }

        // must be gray
        return true;
    }


}












