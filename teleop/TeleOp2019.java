package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="2019", group="Linear Opmode")
@Disabled
public class TeleOp2019 extends LinearOpMode {

    // my robot parts
    // motors
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

    //sensors
    private DigitalChannel armSensor;

    @Override
    public void runOpMode() {
        // Player 1
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        foundation = hardwareMap.get(Servo.class, "foundation");

        // Player 2
        sweepMotorLeft = hardwareMap.get(DcMotor.class, "sweepMotorLeft");
        sweepMotorRight = hardwareMap.get(DcMotor.class, "sweepMotorRight");
        sweepLeft = hardwareMap.get(Servo.class, "sweepLeft");
        sweepRight = hardwareMap.get(Servo.class, "sweepRight");

        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");

        armSensor = hardwareMap.get(DigitalChannel.class, "armSensor");
        armSensor.setMode(DigitalChannel.Mode.INPUT);

        double motorPower = 0.75;

        waitForStart();

        while (opModeIsActive()) {

            // moving blocks in
            if(gamepad2.b) {
                while (gamepad2.b) { // Intake
                    sweepMotorLeft.setPower(0.75);
                    sweepMotorRight.setPower(0.75);
                    sweepLeft.setPosition(0);
                    sweepRight.setPosition(1);
                }
            } else if (gamepad2.a){ // In case of jam
                while(gamepad2.a) {
                    sweepMotorLeft.setPower(-0.75);
                    sweepMotorRight.setPower(-0.75);
                }
            }else if (gamepad2.y){ //Lock in
                while(gamepad2.y) {
                    sweepLeft.setPosition(0);
                    sweepRight.setPosition(1);
                }
            } else {
                sweepMotorLeft.setPower(0);
                sweepMotorRight.setPower(0);
                sweepLeft.setPosition(0.5);
                sweepRight.setPosition(0.5);
            }

            // moves the claw
            if (gamepad2.left_bumper) {
                claw.setPosition(0);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(1);
            }

            // moves arm
            if (gamepad2.right_trigger > 0){
                while(arm.getPosition() > 0.01){
                    arm.setPosition(arm.getPosition() - 0.0005);
                }
            } else {
                arm.setPosition(0.95);
            }

            //lift arm
            if(!armSensor.getState() && gamepad2.left_stick_y > 0) {
                lift.setPower(0);
            } else {
                lift.setPower(gamepad2.left_stick_y);
            }

            if(gamepad1.b) {
                foundation.setPosition(0);
            }

            frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backRight.setPower(motorPower * -(-this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            backLeft.setPower(motorPower * -(this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));

            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power: ", motorPower);
            telemetry.addData( "IsPressed()? ", armSensor.getState());
            telemetry.update();
        }
    }
}