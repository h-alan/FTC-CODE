package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Official Drive Program")
public class OfficialDrive extends LinearOpMode {
    // my robot parts
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    //private Servo servoClaw;
    // Servo servoLift1;
    private Servo swerve;
    private DcMotor liftMotor;
    private DcMotor sweep;
    private CRServo liftArm;
    private Rev2mDistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // sets up all the mecanum wheels
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        //servoLift1 = hardwareMap.get(Servo.class, "servoLift1");
        swerve = hardwareMap.get(Servo.class, "swerve");
        liftArm = hardwareMap.get(CRServo.class, "liftArm");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
        sweep = hardwareMap.get(DcMotor.class, "sweep");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)


        waitForStart();
        // run until the end of the match (driver presses STOP)

        double tgtPower = 0;
        //liftArm.setDirection(Servo.Direction.REVERSE);
        //liftArm.setPosition(0);
        swerve.setDirection(Servo.Direction.REVERSE);
        swerve.setPosition(0);
        while (opModeIsActive()) {
            // movement controls
            frontRight.setPower(0.75 * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            frontLeft.setPower(0.75 * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backRight.setPower(0.75 * (-this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backLeft.setPower(0.75 * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            // check to see if we need to move the serve

            if (gamepad2.b) {
                sweep.setPower(0.75);
            } else if (gamepad2.a) {
                sweep.setPower(-0.75);
            } else {
                sweep.setPower(0);
            }

            // this changes the position of the servo, moving the spinny thing up and down
            if (gamepad2.right_bumper) {
                swerve.setPosition(swerve.getPosition() - 0.05);
            } else if (gamepad2.left_bumper) {
                swerve.setPosition(swerve.getPosition() + 0.05);
            } else {
                swerve.setPosition(swerve.getPosition());
            }


            liftMotor.setPower(-gamepad2.right_stick_y * 0.55);
/*
            if(gamepad2.left_stick_y<0){
                liftArm.setPosition(liftArm.getPosition() + 0.08);//up
            }if(gamepad2.left_stick_y>0) {
                liftArm.setPosition(liftArm.getPosition() - 0.04);//down
            }else{
                liftArm.setPosition(liftArm.getPosition()+0);
            }

            if(liftArm.getPosition()>0.45){
                liftArm.setPosition(0.45);
            }

            if(liftArm.getPosition()<0.25){
                liftArm.setPosition(0.25);
            }*/

            liftArm.setPower(gamepad2.left_stick_y-0.005);

            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", frontRight.getPower());
            telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", distanceSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("LiftArm Position", liftArm.getPosition());


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
