package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private DcMotorEx launcher;

    // servos
    private Servo wobbleArm;
    private Servo launcherPush;  // moves the rings into the launching motor
    private Servo claw;

    double motorPower = 0.75;
    double launcherRPM = 400;

    private double NEW_P = 10.0;
    private double NEW_I = 0.01;
    private double NEW_D = 0;

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
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherPush = hardwareMap.get(Servo.class, "launcherPush");

        wheelRack = hardwareMap.get(DcMotor.class, "wheelRack");
        belt = hardwareMap.get(DcMotor.class, "belt");
        //arm is god servo
        wobbleArm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        wobbleArm.setPosition(1);
        wobbleArm.setPosition(wobbleArm.getPosition());

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        final int motorIndex = ((DcMotorEx)launcher).getPortNumber();
        final DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)launcher.getController();

        /*
        ----------------------------------------------
        */

        // PID increasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean increase = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.y && increase) {
                    NEW_P += 1;
                    increase = false; // won't increase the motor power if the button is held down
                    NEW_P = Math.round(NEW_P * 100.0) / 100.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that

                    launcher.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
                    //motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
                } else if (!gamepad2.y)
                    increase = true; // releasing the bumper  will allow you to increase again
            }


        }));

        // PID decreasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean decrease = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.b && decrease) {
                    NEW_P -= 1;
                    decrease = false; // won't increase the motor power if the button is held down
                    NEW_P = Math.round(NEW_P * 100.0) / 100.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that

                    PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
                    launcher.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
                } else if (!gamepad2.b)
                    decrease = true; // releasing the bumper  will allow you to increase again
            }
        }));

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
                    claw.setPosition(0.35);
                } else {
                    claw.setPosition(1);
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
                    launcher.setVelocity(-launcherRPM/60 * 383.6);
                } else {
                    launcher.setVelocity(0);
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
         *    launcherRPM
         */

        // increasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean increase = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.right_bumper && increase) {
                    launcherRPM += 20;
                    if (launcherRPM > 6000) {
                        launcherRPM = 6000;
                    }
                    increase = false; // won't increase the motor power if the button is held down
                } else if (!gamepad2.right_bumper)
                    increase = true; // releasing the bumper  will allow you to increase again

                launcherRPM = Math.round(launcherRPM * 1000.0) / 1000.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
            }
        }));

        // decreasing
        registerThread(new TaskThread(new TaskThread.Actions() {
            boolean decrease = true; //Outside of loop()

            @Override
            public void loop() {
                if (gamepad2.left_bumper && decrease) {
                    launcherRPM -= 20;
                    if (launcherRPM < 0) {
                        launcherRPM = 0;
                    }
                    decrease = false; // won't decrease the motor power if the button is held down
                } else if (!gamepad2.left_bumper)
                    decrease = true; // releasing the bumper  will allow you to decrease again

                launcherRPM = Math.round(launcherRPM * 1000.0) / 1000.0; // motor power is sometimes wacky and will have infinite decimals, rounding to fix that
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
        telemetry.addData("Launcher Power: ", launcherRPM);
        PIDFCoefficients pidOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f", pidOrig.p, pidOrig.i, pidOrig.d);
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
