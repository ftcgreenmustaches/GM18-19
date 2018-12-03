package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file provides basic Telop driving for a simple mecanum drive robot.
 * This OpMode uses the T_DrivetrainOnlyHardware hardware class to define the devices on the robot.
 * All device access is managed through the T_DrivetrainOnlyHardware class.
 * This particular OpMode executes a code to run the mecanum wheels to allow for them to work correctly
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "ArmTest", group = "FTCRed")
public class ArmTest extends OpMode {
    /* Declare OpMode members. */
    public DcMotor mineralmotorleft = null;
    int lastPos;

    Hardware robot = new Hardware(); // use the class created to define a robot's hardware

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello FTC Red Driver");    //
        mineralmotorleft = hardwareMap.get(DcMotor.class, "mineralmotorleft");
        mineralmotorleft.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        mineralmotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double armPos = mineralmotorleft.getCurrentPosition();
        telemetry.addData("encoder-fdw", armPos + "busy=" + mineralmotorleft.isBusy());
        telemetry.update();


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
                telemetry.addData("Braking", mineralmotorleft.getCurrentPosition());
                mineralmotorleft.setPower(0.02);
                mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

//            while (mineralmotorleft.isBusy()) {
//                telemetry.addData("encoder-fdw", mineralmotorleft.getCurrentPosition()
//                        + "busy=" + mineralmotorleft.isBusy());
//                telemetry.update();
//
//            }

//            mineralmotorleft.setPower(0);

        } else if (gamepad2.y) {
            mineralmotorleft.setTargetPosition(1500);
            mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mineralmotorleft.setPower(0.75);

//            while (mineralmotorleft.isBusy()) {
//                telemetry.addData("encoder-fdw", mineralmotorleft.getCurrentPosition() +
//                        "busy=" + mineralmotorleft.isBusy());
//                telemetry.update();
//            }
//
//
//            mineralmotorleft.setPower(0);

        } else if (gamepad2.b) {
            mineralmotorleft.setTargetPosition(500);
            mineralmotorleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mineralmotorleft.setPower(0.4);

//            while (mineralmotorleft.isBusy()) {
//                telemetry.addData("encoder-fdw", mineralmotorleft.getCurrentPosition() +
//                        "busy=" + mineralmotorleft.isBusy());
//                telemetry.update();
//            }


        } else if (mineralmotorleft.getCurrentPosition() < 1250 && mineralmotorleft.getCurrentPosition() > 200) {
            telemetry.addData("Braking", mineralmotorleft.getCurrentPosition());
            mineralmotorleft.setPower(0.03);
            mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            telemetry.addData("Braking", "not");
            mineralmotorleft.setPower(0);
            mineralmotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }







        /*
         * Code to run ONCE after the driver hits STOP
         */


    }


}












