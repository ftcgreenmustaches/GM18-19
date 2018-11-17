package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
    public Servo TMServo=null;
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
TMServo= hardwareMap.get(Servo.class,"TMServo");
        flipmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipmotor.setTargetPosition(0);

        flipmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        if (gamepad2.a) {
            mineralmotorleft.setPower(0.4);
        } else if (gamepad2.y) {
            mineralmotorleft.setPower(-1);
        } else {
            mineralmotorleft.setPower(0);
        }

        if (gamepad2.left_bumper) {
            spinner.setPower(-.69);
        } else if (gamepad2.right_bumper) {
            spinner.setPower(0.69);
        } else {
            spinner.setPower(0);
        }


if (gamepad1.a){
            TMServo.setPosition(1);
}



        if (gamepad1.y) {
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
        }




        /*
         * Code to run ONCE after the driver hits STOP
         */
    }
}












