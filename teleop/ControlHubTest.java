package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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
            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power: ", motorPower);
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

