package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Extend ThreadOpMode rather than OpMode
@TeleOp(name="Thread", group="threadrd Opmode")
public class ExampleOpMode extends ThreadOpMode {

    //Define global variables
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor sweepMotorLeft;
    private DcMotor sweepMotorRight;
    // servos
    private Servo servoClamp;
    private Servo swerve;
    private Servo sweepLeft;
    private Servo sweepRight;

    final double motorPower = 0.75;

    @Override
    public void mainInit() {
        //Perform your normal init
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        sweepMotorLeft = hardwareMap.get(DcMotor.class, "sweepMotorLeft");
        sweepMotorRight = hardwareMap.get(DcMotor.class, "sweepMotorRight");
        // set up servos
        //servoClamp = hardwareMap.get(Servo.class, "testServo");
        swerve = hardwareMap.get(Servo.class, "swerve");
        sweepLeft = hardwareMap.get(Servo.class, "sweepLeft");
        sweepRight = hardwareMap.get(Servo.class, "sweepRight");

        //Below is a new thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //The loop method should contain what to constantly run in the thread
                //For instance, this drives a single DcMotor
                // moving the arms
                // moving the blocks in
                if(gamepad1.right_bumper) {
                    while (gamepad2.right_bumper) {
                        sweepMotorLeft.setPower(0.75);
                        sweepMotorRight.setPower(0.75);
                        sweepLeft.setPosition(0);
                        sweepRight.setPosition(1);
                    }
                }
                else {
                    sweepMotorLeft.setPower(0);
                    sweepMotorRight.setPower(0);
                    sweepLeft.setPosition(0.5);
                    sweepRight.setPosition(0.5);
                }

            }
        }));

        //Below is a new thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //The loop method should contain what to constantly run in the thread
                //For instance, this drives a single DcMotor
                // this changes the position of the servo, moving the spinny thing up and down
                if (gamepad2.x) {
                    swerve.setPosition(0.0);
                } else if (gamepad2.y) {
                    swerve.setPosition(1);
                }
            }
        }));
    }

    @Override
    public void mainLoop() {
        frontRight.setPower(motorPower * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        frontLeft.setPower(motorPower * (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        backRight.setPower(motorPower * -(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        backLeft.setPower(motorPower * -(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
    }
}
