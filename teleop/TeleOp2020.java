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

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // sweeping motors
    private DcMotor wheelRack;
    private DcMotor belt;

    private DcMotor launcher;

    // servos
    private Servo wobbleArm;
    private Servo launcherPush;


    //sensors
    //private DigitalChannel ringSensor;

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

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcherPush = hardwareMap.get(Servo.class, "launcherPush");

        wheelRack = hardwareMap.get(DcMotor.class, "wheelRack");
        belt = hardwareMap.get(DcMotor.class, "belt");

        // Player 2
        //arm is god servo
        //wobbleArm = hardwareMap.get(Servo.class, "arm");

        /*
        ----------------------------------------------
        */

        // moves arm. GOD SERVO
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad1.right_trigger > .05) {
                    while (wobbleArm.getPosition() > 0.01) {
                        wobbleArm.setPosition(wobbleArm.getPosition() - 0.0040);
                    }
                } else if (gamepad1.left_trigger > .05) {
                    while (wobbleArm.getPosition() < .95) {
                        wobbleArm.setPosition(wobbleArm.getPosition() + 0.0055);
                    }
                }
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                boolean changed = false;
                if (gamepad1.b && !changed) {
                    if (wheelRack.getPower() == 0) {
                        wheelRack.setPower(1);
                        belt.setPower(0.5);
                    } else {
                        wheelRack.setPower(0);
                        belt.setPower(0);
                    }
                    changed = true;
                } else if (!gamepad1.b) changed = false;
            }
        }));

        // launcher controls
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad1.a) {
                    launcher.setPower(1);
                } else {
                    launcher.setPower(0);
                }
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                if (gamepad1.x) {
                    launcherPush.setPosition(0.2);
                } else {
                    launcherPush.setPosition(0);
                }
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
