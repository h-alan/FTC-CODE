package org.firstinspires.ftc.teamcode.autonomous;


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

@Autonomous(name = "Go BACKWARD!!", group = "None")
public class GoBackward extends AutoFrame {

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

        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);

        waitForStart();

        // go o line
        goBackward(0.25);
        while (tof.getDistance(DistanceUnit.MM) > 400) {//lowers the robot
            goBackward(0.25);
            telemetry.addData("range", String.format("%.01f mm", tof.getDistance(DistanceUnit.MM)));
        }
        stopMotors();
        sweepLeft.setPosition(0);
        sweepRight.setPosition(1);
        foundation.setPosition(0.475);

        telemetry.addLine("frik mah life");
        telemetry.update();

    }
}