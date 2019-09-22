// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Drive Avoid Imu")
@Disabled
public class DriveAvoidImu extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Servo servoClaw;
    private Servo servoLift1;
    private Servo swerve;
    private DcMotor liftMotor;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle, power, correction;
    boolean aButton, bButton, touched;


    // called when init button is  pressed.
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        servoLift1 = hardwareMap.get(Servo.class, "servoLift1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        swerve = hardwareMap.get(Servo.class, "swerve");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        while (opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1. imu heading", lastAngles.firstAngle);
            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(lastAngles.angleUnit, lastAngles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(lastAngles.angleUnit, lastAngles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle);
                        }
                    });
            telemetry.addData("2. global heading", globalAngle);
            telemetry.addData("3. correction", correction);
            telemetry.update();


            goForward(0.3, correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;

            if (aButton || bButton) {
                // backup.
                goBackward(0.3, correction);

                sleep(500);

                // stop.
                stopWheels();

                // turn 90 degrees right.
                if (aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        stopWheels();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        // set power to rotate.
        backLeft.setPower(leftPower);
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        frontLeft.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        stopWheels();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // movement methods
    public void goForward(double tgtPower, double correction) {
        frontRight.setPower(-tgtPower + correction);
        frontLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower + correction);
        backLeft.setPower(tgtPower);
    }

    public void goBackward(double tgtPower, double correction) {
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower + correction);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower + correction);
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
        frontLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);
    }

    public void turnLeft(double tgtPower) {
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        frontRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
    }
}
