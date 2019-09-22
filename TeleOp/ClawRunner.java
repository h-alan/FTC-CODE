package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Claw Program")
@Disabled
public class ClawRunner extends LinearOpMode {
    // my robot parts
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Servo servoClaw;
    private Servo liftArm;
    private Servo swerve;
    private DcMotor liftMotor;

    @Override
    public void runOpMode() {
        // sets up all the mecanum wheels
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        liftArm = hardwareMap.get(Servo.class, "liftArm");
        swerve = hardwareMap.get(Servo.class, "swerve");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        double tgtPower = 0;

        liftArm.setDirection(Servo.Direction.REVERSE);
        while (opModeIsActive()) {
            // movement controls
            frontRight.setPower(0.7 * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            frontLeft.setPower(0.7 * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backRight.setPower(0.7 * (-this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backLeft.setPower(0.7 * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            // check to see if we need to move the servo.
            // moves the claw
            if (gamepad2.left_bumper) {
                // move to 45 degrees.
                servoClaw.setPosition(0.25);
            } else if (gamepad2.right_bumper) {
                // move to 180 degrees.
                servoClaw.setPosition(0);
            }

            // sets the lift motor with gamepad2 right stick
            liftMotor.setPower(-gamepad2.right_stick_y * 0.65);

            // this sets the positions of the arm with gamepad2 left stick
            liftArm.setPosition(-gamepad2.left_stick_y / 2 + 0.25);
/*
            if (swerve.getPosition() > (gamepad2.right_trigger + 0.075)) {
                swerve.setPosition(swerve.getPosition() - 0.075);
            } else if (swerve.getPosition() < gamepad2.right_trigger - 0.075) {
                swerve.setPosition(swerve.getPosition() + 0.075);
            } else {
                swerve.setPosition(swerve.getPosition());
            }
            */

            // this moves the claw up and down
            if (gamepad2.a) {
                // move  degrees
                swerve.setPosition(0);
            } else if (gamepad2.b) {
                swerve.setPosition(0.25);
            } else if (gamepad2.y) {
                swerve.setPosition(0.45);
            }

            // the telemetry is added
            telemetry.addData("High torque position", swerve.getPosition());
            telemetry.addData("Servo Position", servoClaw.getPosition());

        /*
            // starting movement
            if (this.gamepad1.left_stick_y < -0.1) {
                goForward(this.gamepad1.left_stick_y);
            } else if (this.gamepad1.left_stick_y > 0.1) {//back
                goBackward(-this.gamepad1.left_stick_y);
            } else if (this.gamepad1.left_stick_x < -0.1) {
                strafeRight(this.gamepad1.left_stick_x);
            } else if (this.gamepad1.left_stick_x > 0.1) {
                strafeLeft(this.gamepad1.left_stick_x);
            } else {
                stopWheels();
            }
            // controlling the right joystick, for turning purposes
            if (gamepad1.right_stick_x > 0.1) {
                turnRight(gamepad1.right_stick_x);
            } else if (gamepad1.right_stick_x < -0.1) {
                turnLeft(gamepad1.right_stick_x);
            }
            */

            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", frontRight.getPower());
            telemetry.addData("Servo Pos", swerve.getPosition());


            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    // movement methods
    /* public void goForward(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(tgtPower);
    }

    public void goBackward(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void stopWheels() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafeRight(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void strafeLeft(double tgtPower) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void turnRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void turnLeft(double tgtPower) {
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
    }

    */


}
