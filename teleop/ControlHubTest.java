package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class ControlHubTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    //Distance Sensors
    private Rev2mDistanceSensor top;
    private Rev2mDistanceSensor bottom;

    double motorPower = 0.75;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Player 1
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        top = hardwareMap.get(Rev2mDistanceSensor.class, "top");
        bottom = hardwareMap.get(Rev2mDistanceSensor.class, "bottom");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x < -0.85) {
                strafeLeft(motorPower);
            } else if (gamepad1.left_stick_x > 0.85) {
                strafeRight(motorPower);
            } else {
                frontRight.setPower(motorPower * (-this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                frontLeft.setPower(motorPower * (this.gamepad1.left_stick_y - this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                backRight.setPower(motorPower * (-this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
                backLeft.setPower(motorPower * (this.gamepad1.left_stick_y + this.gamepad1.left_stick_x - this.gamepad1.right_stick_x));
            }
            int rings;
            if(bottom.getDistance(DistanceUnit.MM) < 200) {
                if (top.getDistance(DistanceUnit.MM) < 200) {
                    rings = 4;
                }
                else {
                    rings = 1;
                }
            }
            else{
                rings = 0;
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power: ", motorPower);
            telemetry.addData("Top Distance:", top.getDistance(DistanceUnit.MM));
            telemetry.addData("Bottom Distance:", bottom.getDistance(DistanceUnit.MM));
            telemetry.addData("Rings:", rings);
            telemetry.update();
        }
    }

    private void strafeRight(double tgtPower) {
        frontRight.setPower(-tgtPower);
        frontLeft.setPower(tgtPower);
        backRight.setPower(tgtPower);
        backLeft.setPower(-tgtPower);
    }

    private void strafeLeft(double tgtPower) {
        frontLeft.setPower(-tgtPower);
        frontRight.setPower(tgtPower);
        backLeft.setPower(tgtPower);
        backRight.setPower(-tgtPower);
    }
}

