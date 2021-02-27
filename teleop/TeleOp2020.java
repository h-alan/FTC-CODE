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

import static java.lang.Thread.sleep;

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
    double launcherRPM = 250;

    private double NEW_P = 150.0;
    private double NEW_I = 0;
    private double NEW_D = 0;

    //power shot stuff
    static final double powerShot2 = 8;
    static final double powerShot3 = 7;

    double goForBakDistanceCorrection = 1.10;  //Calibration showed setting to go forward/backward 32 inches command, but actually achieved 29 inches when motor power was set as 0.6.
    double goStrafeDistanceCorrection = 1.34;  //Calibration showed setting to strafe left/right 32 inches command, but actually achieved 24 inches when motor power was set as 0.6.

    double launcherVelocity = 225;
    double motorPowerAuto = 0.4;  // set the vehicle DC motor power.

    static final double COUNTS_PER_MOTOR_REV = 383.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP... maybe 1??
    static final double WHEEL_DIAMETER_INCHES = 10 / 2.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (12 / 9.6)/* (12/10.25) */ * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .1;
    static final double TURN_SPEED = 0.5;
    static final double ringDistance = 39; // distance from origin to the location of ring detection (inch)

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

        launcher.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);

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
            boolean increase = true; //Outside of loop()
            boolean launch = false;
            @Override
            public void loop() {
                if (gamepad2.left_trigger > 0.85 && increase) {
                    //Increase launcher power
                    if(!launch) {
                        launcher.setVelocity(-(launcherRPM/60 * 383.6));
                        launch = true;
                    }
                    else {
                        launcher.setVelocity(0);
                        launch = false;
                    }
                    increase = false; // won't increase the motor power if the button is held down
                } else if (gamepad2.left_trigger < 0.85) {
                    increase = true; // releasing the bumper  will allow you to increase again
                }
            }
        }));

        // servo
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad2.right_trigger > 0.85) {
                    for (int i = 0; i < 3; i++) {
                        launcherPush.setPosition(0.5);

                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        launcherPush.setPosition(0.7);

                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
                /*
                else {
                    launcherPush.setPosition(0.7);
                }
                 */
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
                    launcherRPM += 10;
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
                    launcherRPM -= 10;
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
        telemetry.addData("Launcher RPM: ", launcherRPM);
        PIDFCoefficients pidOrig = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f", pidOrig.p, pidOrig.i, pidOrig.d);
        telemetry.addData("Velocity: ", launcher.getVelocity() / 383.6 * 60);
        telemetry.update();

        if (gamepad2.b) {
            launcherPush.setPosition(0.7);
            launcher.setVelocity(-launcherVelocity / 60 * 383.6);
            //stopMotors();
/*                    rotate((int)(0 - getAngle()),0.1);
                    resetAngle();*/
            //Starts shooting
            //launcher.setVelocity(-250 / 60 * 383.6);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
/*                    rotate((int)(0 - getAngle()),0.1);
                    resetAngle();*/
            launcherPush.setPosition(0.3);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launcherPush.setPosition(0.7);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            telemetry.addData("Velocity: ", launcher.getVelocity() / 383.6 * 60);
            strafeLeftEncoder(powerShot2*goStrafeDistanceCorrection, motorPowerAuto);
/*                    rotate((int)(0 - getAngle()),0.1);
                    resetAngle();*/
            launcherPush.setPosition(0.3);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launcherPush.setPosition(0.7);
            telemetry.addData("Velocity: ", launcher.getVelocity() / 383.6 * 60);
            strafeLeftEncoder(powerShot3*goStrafeDistanceCorrection, motorPowerAuto);
/*                    rotate((int)(0 - getAngle()),0.1);
                    resetAngle();*/
            launcherPush.setPosition(0.3);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launcherPush.setPosition(0.7);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            telemetry.addData("Velocity: ", launcher.getVelocity() / 383.6 * 60);
            launcher.setVelocity(0);
        }
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

    protected void encoderDrive(double speed,
                                double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches,
                                double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        //double adjustRT = 1.01; // Right distance adjustment 1% more than left
        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30; // Start at this power
        final double SPEEDINCR = 0.2; // And increment by this much each cycle
        double curSpeed; // Keep track of speed as we ramp

        // Ensure that the opmode is still active
            // Determine new target position, and pass to motor controller
            //DUE TO ORIENTATION OF MOTORS, LEFT MOTORS HAVE TO HAVE SIGN REVERSED FOR DISTANCES. maybe.
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (-FrontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (-BackLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (BackRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            while (frontLeft.getTargetPosition() != newFrontLeftTarget) {
                frontLeft.setTargetPosition(newFrontLeftTarget);
                try {
                    sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            while (frontRight.getTargetPosition() != newFrontRightTarget) {
                frontRight.setTargetPosition(newFrontRightTarget);
                try {
                    sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            while (backLeft.getTargetPosition() != newBackLeftTarget) {
                backLeft.setTargetPosition(newBackLeftTarget);
                try {
                    sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            while (backRight.getTargetPosition() != newBackRightTarget) {
                backRight.setTargetPosition(newBackRightTarget);
                try {
                    sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }


            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.

            curSpeed = Math.abs(speed); // Make sure its positive
            //curSpeed = Math.min(MINSPEED, speed);


            frontLeft.setPower(curSpeed);
            backLeft.setPower(curSpeed);
            frontRight.setPower(curSpeed);
            backRight.setPower(curSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy()&& backRight.isBusy())) { //might need to change this to all being busy..
/*
                    // Ramp up motor powers as needed
                    if (Math.abs(frontLeft.getCurrentPosition()- newFrontLeftTarget) < (0.3*Math.abs(newFrontLeftTarget)))
                    {
                        telemetry.addData("test", "qao hi");
                        if (curSpeed > MINSPEED) {
                            curSpeed -= SPEEDINCR;
                        }
                        else curSpeed = MINSPEED;

                    }
                    else {
                        // Ramp up motor powers as needed
                        if (curSpeed < speed) {
                            curSpeed += SPEEDINCR;
                        } else curSpeed = speed;
                    }

 */
                // And rewrite the motor speeds
                frontLeft.setPower(curSpeed);
                backLeft.setPower(curSpeed);
                frontRight.setPower(curSpeed);
                backRight.setPower(curSpeed);


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d : %7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            try {
                sleep(100);   // optional pause after each move
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

    }

    private void strafeLeftEncoder(double strafeLeftInch, double mymotorPower) {
        //frontRight.setPower(-tgtPower);
        //frontLeft.setPower(-tgtPower);
        //backRight.setPower(tgtPower);
        //backLeft.setPower(tgtPower);
        encoderDrive(mymotorPower, -strafeLeftInch, strafeLeftInch, strafeLeftInch, -strafeLeftInch, 30);
    }
}
