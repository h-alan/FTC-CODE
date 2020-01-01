package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test", group="Linear Opmode")
@Disabled
public class TestCode extends LinearOpMode {
    private Servo swerve;

    @Override
    public void runOpMode() {
        // set up servos
        swerve = hardwareMap.get(Servo.class, "swerve");

        double motorPower = 0.75;

        waitForStart();

        while (opModeIsActive()) {

            // this changes the position of the servo, moving the spinny thing up and down
            if (gamepad2.x) {
                swerve.setPosition(1);
            } else if (gamepad2.y) {
                swerve.setPosition(0);
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("Motor Power: ", motorPower);
            telemetry.update();
        }
    }
}
