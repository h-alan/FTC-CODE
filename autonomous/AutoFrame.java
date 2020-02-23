/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class AutoFrame extends LinearOpMode {
    /*
    -------------ROBOT PARTS SETUP---------------
     */
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor backRight;
    protected DcMotor backLeft;
    protected DcMotor sweepMotorLeft;
    protected DcMotor sweepMotorRight;
    protected DcMotor lift;

    // servos
    protected Servo arm;
    protected Servo claw;
    protected Servo sweepLeft;
    protected Servo sweepRight;
    protected Servo foundation;
    protected Servo stoneGrabber;

    //sensors
    protected DigitalChannel armSensor;
    protected Rev2mDistanceSensor tof;
    protected Rev2mDistanceSensor fronttof;

    final double motorPower = 0.75;

    // imu stuff
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    /*
    --------------------------------------
    */

    /*
      -------------VUFORIA SETUP----------
    */
    protected static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    protected static final boolean PHONE_IS_PORTRAIT = false;

    protected static final String VUFORIA_KEY =
            "AeflKlH/////AAABmQAedZNXCkKmqQ2CkC55GVkZGoxK0YlVMNeDwQgN5B9Jq26R9J8TZ0qlrBQVz2o3vEgIjMfV8rZF2Z7PPxZJnScBap/Jh2cxT0teLCWkuBk/mZzWC0bRjhpwT0JkU3AGpztJHL4oJZDEaf4fUDilG1NdczNT5V8nL/ZraZzRZvGBwYO7q42b32DKKb+05OemiCOCx34h0qq0lkahDKKO7k1UTpznzyK33IPVtvutSgGvdrpNe/Jv5ApIvHcib4bKom7XVqf800+Adi0bDD94NSWFeJq+i/IZnJJqH9iXXdl3Qjptri6irrciVJtmjtyZCnFB0n4ni90VmmDb5We3Dvft6wjdPrVO5UVAotWZJAnr";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    protected static final float mmPerInch = 25.4f;
    protected static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    protected static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    protected static final float bridgeZ = 6.42f * mmPerInch;
    protected static final float bridgeY = 23 * mmPerInch;
    protected static final float bridgeX = 5.18f * mmPerInch;
    protected static final float bridgeRotY = 59;                                 // Units are degrees
    protected static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    protected static final float halfField = 72 * mmPerInch;
    protected static final float quadField = 36 * mmPerInch;

    // Class Members
    protected OpenGLMatrix lastLocation = null;
    protected VuforiaLocalizer vuforia = null;
    protected boolean targetVisible = false;
    protected float phoneXRotate = 0;
    protected float phoneYRotate = 0;
    protected float phoneZRotate = 0;

    /*
    ----------------------------imu methods----------------------------
     */
    /*
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /*
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    protected double getAngle() {

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

    /*
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    protected double checkDirection() {
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
    -----------------------------------------------------------
     */

    /*
    ----------------------- movement methods ------------------
     */

    protected void moveStone() {
        // move toward stone
        strafeLeft(0.5);
        sleep(900);
        stopMotors();
        sleep(150);
        // latch onto stone
        latchStone();
        sleep(400);

        rotate((int)(0 - getAngle()),1);
        sleep(150);
        /////////MOVING TO OTHER SIDE//////////
        strafeRight(1);
        sleep(700);
        //rotate((int)(0 - getAngle()),1);
        sleep(150);
    }

    protected void rotate(int degrees, double Power) {
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
        sleep(100);
        return;
    }

    protected void collect(int numTimes) {
        for (int i = 0; i < numTimes; i++) {
            intake(0.75);
            sleep(600);
            outtake();
            sleep(600);
        }
    }

    protected void moveArm() {
        while (arm.getPosition() > 0.01) {
            arm.setPosition(arm.getPosition() - 0.0095);
        }
    }

    protected void resetArm() {
        arm.setPosition(0.95);
    }

    protected void raiseArm(double tgtPower) {
        lift.setPower(tgtPower);
    }

    protected void lowerArm() {
        if (!armSensor.getState()) {
            lift.setPower(0);
        }
    }

    protected void intake(double tgtPower) {
        sweepMotorLeft.setPower(0.75);
        sweepMotorRight.setPower(0.75);
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
    }

    protected void releaseStone(double tgtPower) {
        sweepMotorLeft.setPower(-tgtPower);
        sweepMotorRight.setPower(-tgtPower);
    }

    protected void outtake() {
        sweepMotorLeft.setPower(0);
        sweepMotorRight.setPower(0);
        sweepLeft.setPosition(0.5);
        sweepRight.setPosition(0.5);
    }

    protected void lockIn() {
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
    }

    protected void closeClaw() {
        claw.setPosition(1);
    }

    protected void openClaw() {
        claw.setPosition(0);
    }

    protected void latchStone() {
        stoneGrabber.setPosition(0);
    }

    protected void unlatchStone() {
        stoneGrabber.setPosition(1);
    }

    protected void goForward(double tgtPower) { //done?
        frontRight.setPower(tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(tgtPower);
    }

    protected void goBackward(double tgtPower) { //done
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }

    ////////////////////////////////////////////////
    protected void strafeRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(-tgtPower);
        backRight.setPower(-tgtPower);
        backLeft.setPower(-tgtPower);
    }

    protected void strafeLeft(double tgtPower) {
        frontLeft.setPower(tgtPower);
        frontRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);

    }

    protected void stopMotors() {
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

    protected void listhardware() {
        telemetry.setAutoClear(false);

        Iterator<com.qualcomm.robotcore.hardware.HardwareDevice> t = hardwareMap.iterator();
        while (t.hasNext()) {

            telemetry.addData("device found", (t.next().getDeviceName()));
            telemetry.update();
        }

        telemetry.setAutoClear(true);
    }
}