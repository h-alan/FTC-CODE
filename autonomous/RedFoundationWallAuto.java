package org.firstinspires.ftc.teamcode.autonomous.red;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Iterator;

import detectors.FoundationPipeline.SkyStone;
import detectors.OpenCvDetector;

@Autonomous(name = "RED Foundation (WALL)", group = "RedSide")
public class RedFoundationWallAuto extends LinearOpMode {
    /*
    -------------ROBOT PARTS SETUP---------------
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
    private Servo stoneGrabber;

    //sensors
    private DigitalChannel armSensor;
    private Rev2mDistanceSensor tof;

    final double motorPower = 0.75;

    // imu stuff
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    /*
    --------------------------------------
    */

    public void runOpMode() throws InterruptedException {

    /*
        ----------------- hardware setup ----------------------
        */
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        sweepMotorLeft = hardwareMap.get(DcMotor.class, "sweepMotorLeft");
        sweepMotorRight = hardwareMap.get(DcMotor.class, "sweepMotorRight");
        sweepLeft = hardwareMap.get(Servo.class, "sweepLeft");
        sweepRight = hardwareMap.get(Servo.class, "sweepRight");

        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        stoneGrabber = hardwareMap.get(Servo.class, "stoneGrabber");
        tof = hardwareMap.get(Rev2mDistanceSensor.class, "tof");
        foundation = hardwareMap.get(Servo.class, "foundation");

        telemetry.setAutoClear(true);

        telemetry.addData("Booting Up", " . . .");
        telemetry.update();
        /*
        --------------------------------------
        */

        //------------------------IMU SETUP------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // caliblrating imu
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        /**///------------------------------------------------------------------------------------
        // wait for start command.

        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);

        waitForStart();

        foundation.setPosition(0.95);
        strafeLeft(1);
        sleep(300);

        goBackward(1);
        sleep(650);
        stopMotors();
        sleep(400);

        foundation.setPosition(0.475);
        sleep(600);

        rotate(-15, 1.2);
        goForward(0.5);
        sleep(800);

        rotate(-69 - 5, 1.2); // nice
        stopMotors();

        // to wall
        goBackward(0.5);
        sleep(550);
        stopMotors();
        foundation.setPosition(0.95);

        strafeLeft(0.5);
        sleep(2600);
        stopMotors();

        // go to line
        goForward(0.5);
        while (tof.getDistance(DistanceUnit.MM) > 400) {//lowers the robot
            goForward(0.5);
            telemetry.addData("range", String.format("%.01f mm", tof.getDistance(DistanceUnit.MM)));
        }
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
        foundation.setPosition(0.475);
        stopMotors();

        telemetry.addLine("frik mah life");
        telemetry.update();

    }


    /*
    ----------------------------imu methods----------------------------
     */
    /*
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /*
    ----------------------- movement methods ------------------
     */


    public void moveStone() {
        // move toward stone
        strafeLeft(0.5);
        sleep(900);
        stopMotors();
        sleep(150);
        // latch onto stone
        latchStone();
        sleep(900);

        rotate((int)(0 - getAngle()),1);
        sleep(150);
        /////////MOVING TO OTHER SIDE//////////
        strafeRight(1);
        sleep(700);
        rotate((int)(0 - getAngle()),1);
        sleep(150);
    }

    private void rotate(int degrees, double Power) {
        resetAngle();
        stopMotors();
        double tgtPower = Power / 4;
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).

        if (degrees > 0) {   // turn left
            frontLeft.setPower(tgtPower);
            backLeft.setPower(-tgtPower);
            frontRight.setPower(tgtPower);
            backRight.setPower(-tgtPower);
            while (getAngle() < degrees) {
                sleep(1);
            }
        } else if (degrees < 0) {   // turn right
            frontLeft.setPower(-tgtPower);
            backLeft.setPower(tgtPower);
            frontRight.setPower(-tgtPower);
            backRight.setPower(tgtPower);
            while (getAngle() > degrees) {
                sleep(1);
            }

        } else return;

        stopMotors();
        sleep(400);
        return;
    }

    public void collect(int numTimes) {
        for (int i = 0; i < numTimes; i++) {
            intake(0.75);
            sleep(600);
            outtake();
            sleep(600);
        }
    }

    public void moveArm() {
        while (arm.getPosition() > 0.01) {
            arm.setPosition(arm.getPosition() - 0.0095);
        }
    }

    public void resetArm() {
        arm.setPosition(0.95);
    }

    public void raiseArm(double tgtPower) {
        lift.setPower(tgtPower);
    }

    public void lowerArm() {
        if (!armSensor.getState()) {
            lift.setPower(0);
        }
    }

    public void intake(double tgtPower) {
        sweepMotorLeft.setPower(0.75);
        sweepMotorRight.setPower(0.75);
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
    }

    public void releaseStone(double tgtPower) {
        sweepMotorLeft.setPower(-tgtPower);
        sweepMotorRight.setPower(-tgtPower);
    }

    public void outtake() {
        sweepMotorLeft.setPower(0);
        sweepMotorRight.setPower(0);
        sweepLeft.setPosition(0.5);
        sweepRight.setPosition(0.5);
    }

    public void lockIn() {
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void latchStone() {
        stoneGrabber.setPosition(0);
    }

    public void unlatchStone() {
        stoneGrabber.setPosition(1);
    }

    public void goForward(double tgtPower) { //done?
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(tgtPower);
    }

    public void goBackward(double tgtPower) { //done
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }

    ////////////////////////////////////////////////
    public void strafeRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void strafeLeft(double tgtPower) {
        frontLeft.setPower(tgtPower);
        frontRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);

    }

    public void stopMotors() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        sweepMotorLeft.setPower(0);
        sweepMotorRight.setPower(0);
    }

    /*
    ---------------------------------------------
     */

    public void listhardware() {
        telemetry.setAutoClear(false);

        Iterator<HardwareDevice> t = hardwareMap.iterator();
        while (t.hasNext()) {

            telemetry.addData("device found", (t.next().getDeviceName()));
            telemetry.update();
        }

        telemetry.setAutoClear(true);
    }
}