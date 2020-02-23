package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.thread.TaskThread;
import org.firstinspires.ftc.teamcode.thread.ThreadOpMode;

//Extend ThreadOpMode rather than OpMode
@TeleOp(name = "Real TeleOp 2020", group = "Threaded Opmode")
public class TeleOp2019 extends ThreadOpMode {

    /*
    --------------------------------------
     */
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor sweepMotorLeft;
    private DcMotor sweepMotorRight;
    private DcMotor lift;

    // servos
    private Servo arm;
    private Servo claw;
    private Servo sweepLeft;
    private Servo sweepRight;
    private Servo foundation;
    private Servo capStone;

    private Servo stoneGrabber;
    private Servo stoneClamp;

    //sensors
    private DigitalChannel armSensor;

    double motorPower = 0.75;
    /*
    --------------------------------------
    */


    @Override
    public void mainInit() {
        /*
        -----------------Hardware setup--------------
        */

        // Player 1
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        foundation = hardwareMap.get(Servo.class, "foundation");

        // Player 2
        sweepMotorLeft = hardwareMap.get(DcMotor.class, "sweepMotorLeft");
        sweepMotorRight = hardwareMap.get(DcMotor.class, "sweepMotorRight");
        sweepLeft = hardwareMap.get(Servo.class, "sweepLeft");
        sweepRight = hardwareMap.get(Servo.class, "sweepRight");

        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        capStone = hardwareMap.get(Servo.class, "capStone");

        armSensor = hardwareMap.get(DigitalChannel.class, "armSensor");
        armSensor.setMode(DigitalChannel.Mode.INPUT);

        /*
        ----------------------------------------------
        */

        // moving blocks in
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.b) {
                    while (gamepad2.b) { // Intake Right
                        sweepMotorRight.setPower(0.75);
                        sweepRight.setPosition(1);
                    }
                } else if (gamepad2.a) { // Intake both
                    while (gamepad2.a) {
                        sweepLeft.setPosition(0);
                        sweepRight.setPosition(1);
                        sweepMotorLeft.setPower(0.75);
                        sweepMotorRight.setPower(0.75);
                    }
                } else if (gamepad2.y) { //Lock in
                    while (gamepad2.y) {
                        sweepLeft.setPosition(0);
                        sweepRight.setPosition(1);
                    }
                } else if (gamepad2.x) { // intake left
                    sweepLeft.setPosition(0);
                    sweepMotorLeft.setPower(0.75);
                }
                else {
                    sweepMotorLeft.setPower(0);
                    sweepMotorRight.setPower(0);
                    sweepLeft.setPosition(0.5);
                    sweepRight.setPosition(0.5);
                }

            }
        }));

        // moves the claw
        registerThread(new TaskThread(new TaskThread.Actions() {
            /// change thizs
            @Override
            public void loop() {
                if (gamepad2.left_bumper) {
                    claw.setPosition(0.3);
                } else if (gamepad2.right_bumper) {
                    claw.setPosition(1);
                }
            }
        }));

        // moves arm
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.right_trigger > 0) {
                    while (arm.getPosition() > 0.01) {
                        arm.setPosition(arm.getPosition() - 0.0040);
                    }
                } else {
                    while (arm.getPosition() < 0.95) {
                        arm.setPosition(arm.getPosition() + 0.0055);
                    }
                }
            }
        }));

        // lift arm & foundation
        registerThread(new TaskThread(new TaskThread.Actions() {
            // chanrfe this
            @Override
            public void loop() {
                if (!armSensor.getState() && gamepad2.left_stick_y > 0) {
                    lift.setPower(0);
                } else {
                    lift.setPower(gamepad2.left_stick_y);
                }

                if (gamepad1.b) {
                    foundation.setPosition(0.475); // down
                } else if (gamepad1.a) {
                    foundation.setPosition(0.95);   // up
                }

                if (gamepad2.right_stick_button) {
                    capStone.setPosition(0);
                }

            }
        }));

        // motorPower
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad1.right_bumper) {
                    motorPower += 0.25;
                    if (motorPower > 1) {
                        motorPower = 1;
                    }
                    for(int i = 0; i < 50000000; i++) { motorPower = motorPower;}
                } else if (gamepad1.left_bumper) {
                    motorPower -= 0.25;
                    if (motorPower < 0) {
                        motorPower = 0;
                    }
                    for(int i = 0; i < 50000000; i++) {motorPower = motorPower;}
                }
                motorPower = Math.round(motorPower * 100.0) / 100.0;
            }
        }));
    }


    // moving motors and checking for strafes
    @Override
    public void mainLoop() {
        if (gamepad1.left_stick_x < -0.85) {
            strafeLeft(motorPower);
        } else if (gamepad1.left_stick_x > 0.85) {
            strafeRight(motorPower);
        } else {
            frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backRight.setPower(motorPower * -(-this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backLeft.setPower(motorPower * -(this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
        }
        telemetry.addData("Status", "Running");
        telemetry.addData("Motor Power: ", motorPower);
        telemetry.update();
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontLeft.setPower(tgtPower);
        frontRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);

    }
}
