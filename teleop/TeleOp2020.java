package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.thread.TaskThread;
import org.firstinspires.ftc.teamcode.thread.ThreadOpMode;

//Extend ThreadOpMode rather than OpMode
//Copied and Pasted TeleOp2019
@TeleOp(name = "Real TeleOp 2020", group = "Threaded Opmode")
public class TeleOp2020 extends ThreadOpMode {

    /*
    --------------------------------------
     */

    // moving motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // sweeping motors
    private DcMotor wheelRack;
    private DcMotor belt;

    // launching motor
    private DcMotor launcher;

    // servos
    private Servo wobbleArm;
    private Servo launcherPush;  // moves the rings into the launching motor
    private Servo claw;

    double motorPower = 0.75;
    double launcherPower = 0.625;
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

        // Player 2
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcherPush = hardwareMap.get(Servo.class, "launcherPush");

        wheelRack = hardwareMap.get(DcMotor.class, "wheelRack");
        belt = hardwareMap.get(DcMotor.class, "belt");
        //arm is god servo
        wobbleArm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        wobbleArm.setPosition(1);
        wobbleArm.setPosition(wobbleArm.getPosition());
        /*
        ----------------------------------------------
        */

        // moves arm. GOD SERVO
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.x) {
                    while (wobbleArm.getPosition() < .9) {
                        wobbleArm.setPosition(wobbleArm.getPosition() + 0.0055);
                    }

                } else {
                    while (wobbleArm.getPosition() > 0.35) {
                        wobbleArm.setPosition(wobbleArm.getPosition() - 0.0040);
                    }
                }
            }
        }));

        // open and close claw
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.a) {
                    claw.setPosition(1);
                } else {
                    claw.setPosition(0.5);
                }
            }
        }));

        // collecting rings through the belt
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.left_stick_y > 0.2) {
                    wheelRack.setPower(-1);
                    belt.setPower(-0.5);
                } else if (gamepad2.left_stick_y < -0.2) {
                    wheelRack.setPower(1);
                    belt.setPower(0.5);
                } else {
                    wheelRack.setPower(0);
                    belt.setPower(0);
                }
            }
        }));

        /*
         *   launcher controls
         */

        // motor
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.left_trigger > 0.85) {
                    launcher.setPower(-launcherPower);
                } else {
                    launcher.setPower(0);
                }
            }
        }));

        // servo
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.right_trigger > 0.85) {
                    launcherPush.setPosition(0.4);
                } else {
                    launcherPush.setPosition(0.7);
                }
            }
        }));
        /*
         */

        /*
         *    motorPower
         */

        // increasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean increase = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad1.right_bumper && increase) {
                    motorPower += 0.25;
                    if (motorPower > 1) {
                        motorPower = 1;
                    }
                    increase = false; // won't increase the motor power if the button is held down
                } else if (!gamepad1.right_bumper)
                    increase = true; // releasing the bumper  will allow you to increase again

                motorPower = Math.round(motorPower * 100.0) / 100.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
            }
        }));

        // decreasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean decrease = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad1.left_bumper && decrease) {
                    motorPower -= 0.25;
                    if (motorPower < 0) {
                        motorPower = 0;
                    }
                    decrease = false; // won't decrease the motor power if the button is held down
                } else if (!gamepad1.left_bumper)
                    decrease = true; // releasing the bumper  will allow you to decrease again

                motorPower = Math.round(motorPower * 100.0) / 100.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
            }
        }));

        /*
         */


        /*
         *    launcherPower
         */

        // increasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean increase = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.right_bumper && increase) {
                    launcherPower += 0.025;
                    if (launcherPower > 1) {
                        launcherPower = 1;
                    }
                    increase = false; // won't increase the motor power if the button is held down
                } else if (!gamepad2.right_bumper)
                    increase = true; // releasing the bumper  will allow you to increase again

                launcherPower = Math.round(launcherPower * 1000.0) / 1000.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
            }
        }));

        // decreasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean decrease = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.left_bumper && decrease) {
                    launcherPower -= 0.025;
                    if (launcherPower < 0) {
                        launcherPower = 0;
                    }
                    decrease = false; // won't decrease the motor power if the button is held down
                } else if (!gamepad2.left_bumper)
                    decrease = true; // releasing the bumper  will allow you to decrease again

                launcherPower = Math.round(launcherPower * 1000.0) / 1000.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
            }
        }));

        /*
         */
    }

    /*
     *    Robot Movement
     */
    @Override
    public void mainLoop() {
        if (gamepad1.left_stick_x < -0.85) {
            strafeRight(motorPower);
        } else if (gamepad1.left_stick_x > 0.85) {
            strafeLeft(motorPower);
        } else {
            frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y -this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backRight.setPower(motorPower * -(this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
            backLeft.setPower(motorPower * -(-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x + this.gamepad1.right_stick_x));
        }
        telemetry.addData("Motor Power: ", motorPower);
        telemetry.addData("Launcher Power: ", launcherPower);
        telemetry.update();
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }
}
