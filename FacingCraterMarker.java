package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Facing Crater Marker")
@Disabled
public class FacingCraterMarker extends LinearOpMode {
    // my robot parts
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Servo servoClaw;
    private Servo swerve;
    private DcMotor liftMotor;
    private Servo liftArm;
    private Servo markerServo;
    private Rev2mDistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // sets up all the mecanum wheels
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        //servoLift1 = hardwareMap.get(Servo.class, "servoLift1");
        swerve = hardwareMap.get(Servo.class, "swerve");
        markerServo = hardwareMap.get(Servo.class, "markerServo");
        liftArm = hardwareMap.get(Servo.class, "liftArm");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        boolean latch = true;
        swerve.setDirection(Servo.Direction.REVERSE);
        markerServo.setPosition(0.5);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while(swerve.getPosition()<0.2){
            swerve.setPosition(swerve.getPosition()+0.005);
        }
        sleep(1000);
        while(swerve.getPosition()>0.05){
            swerve.setPosition(swerve.getPosition()-0.005);
        }
        sleep(1000);
        while (latch) {
            if (distanceSensor.getDistance(DistanceUnit.MM) > 120) {
                liftMotor.setPower(-0.25);
            } else {
                latch = false;
                liftMotor.setPower(0);
            }
        }
        liftMotor.setPower(0);

        strafeRight(-0.5);
        sleep(500);
        stopWheels();
        sleep(100);

        goForward(-0.5);
        sleep(500);
        stopWheels();

        sleep(500);
        turnLeft(-0.5);
        sleep(1400);
        stopWheels();

        sleep(500);
        goForward(-0.5);
        sleep(250);
        stopWheels();

        sleep(500);
        strafeLeft(-0.5);
        sleep(1000);
        stopWheels();

        sleep(500);
        goForward(-0.5);
        sleep(600);
        stopWheels();

        sleep(500);
        strafeLeft(-0.5);
        sleep(5000);
        stopWheels();

        markerServo.setPosition(1);

        strafeRight(2000);


        //telemetry.addData("High torque position", servoLift1.getPosition());
        telemetry.addData("Servo Position", servoClaw.getPosition());
        telemetry.addData("Motor Power", frontRight.getPower());
        telemetry.addData("deviceName", distanceSensor.getDeviceName());
        telemetry.addData("range", String.format("%.01f mm", distanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", distanceSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", distanceSensor.getDistance(DistanceUnit.INCH)));

        // Rev2mDistanceSensor specific methods.
        telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
        telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

        telemetry.addData("Status", "Running");
        telemetry.update();
    }


    // movement methods
    public void goForward(double tgtPower) {
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
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }

    public void turnRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    public void turnLeft(double tgtPower) {
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
    }

}