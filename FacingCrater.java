//need to finish

package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.List;

@Autonomous(name = "Facing Crater")
public class FacingCrater extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private Servo swerve;
    private DcMotor liftMotor;
    private CRServo liftArm;
    private Servo markerServo;
    private Rev2mDistanceSensor distanceSensor;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final int MINERAL_X_THRESHOLD = 300;
    private static final int GLD_MINERAL_Y_THRESHOLD_1 = 550;
    private static final int GLD_MINERAL_Y_THRESHOLD_2 = 850;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeflKlH/////AAABmQAedZNXCkKmqQ2CkC55GVkZGoxK0YlVMNeDwQgN5B9Jq26R9J8TZ0qlrBQVz2o3vEgIjMfV8rZF2Z7PPxZJnScBap/Jh2cxT0teLCWkuBk/mZzWC0bRjhpwT0JkU3AGpztJHL4oJZDEaf4fUDilG1NdczNT5V8nL/ZraZzRZvGBwYO7q42b32DKKb+05OemiCOCx34h0qq0lkahDKKO7k1UTpznzyK33IPVtvutSgGvdrpNe/Jv5ApIvHcib4bKom7XVqf800+Adi0bDD94NSWFeJq+i/IZnJJqH9iXXdl3Qjptri6irrciVJtmjtyZCnFB0n4ni90VmmDb5We3Dvft6wjdPrVO5UVAotWZJAnr";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        // sets up all the mecanum wheels
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        swerve = hardwareMap.get(Servo.class, "swerve");
        markerServo = hardwareMap.get(Servo.class, "markerServo");
        liftArm = hardwareMap.get(CRServo.class, "liftArm");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        telemetry.addData("Status", "Initialized");

        while(swerve.getPosition()<0.95){
            swerve.setPosition(swerve.getPosition()+0.005);
        }

        markerServo.setPosition(0.5);

        ///////////////////////////////////////////////liftMotor.setPower(0.55);

        waitForStart();

        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }
        markerServo.setPosition(0.5);//1  to unhook marker
        boolean latch = true;
        swerve.setDirection(Servo.Direction.REVERSE);//important stuff

        waitForStart();
        sleep(1000);
        while(swerve.getPosition()>0.05){
            swerve.setPosition(swerve.getPosition()-0.005);
        }
        sleep(1000);

        char cubepos = 'n';
        for(int i = 0; i < 9001; i++) {//repeats 100 times to rid "white noise"
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    //if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    int objectX = 0;

                    // the following for loop goes through all objects detected
                    for (Recognition recognition : updatedRecognitions) {
                        // get the objects that are outside the cradle
                        objectX = (int) recognition.getLeft();

                        // process objects outside the cradle, otherwise move on to next object
                        if (objectX < MINERAL_X_THRESHOLD) {
                            // only check gold minerals, skip others
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getTop();
                            }
                        }
                    }

                    telemetry.addData("Gold X", goldMineralX);
                    telemetry.addData("Silver 1 X", silverMineral1X);
                    telemetry.addData("Silver 2 X", silverMineral2X);

                    if (goldMineralX != -1) {
                        if (goldMineralX <GLD_MINERAL_Y_THRESHOLD_1) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            cubepos = 'l';
                        } else if (goldMineralX < GLD_MINERAL_Y_THRESHOLD_2 && goldMineralX >= GLD_MINERAL_Y_THRESHOLD_1) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            cubepos = 'c';
                        } else {
                            telemetry.addData("Gold Mineral Position", "Right");
                            cubepos = 'r';
                        }

                        telemetry.update();
                    } else {
                        telemetry.addData("Gold Mineral Position", "Right");
                        cubepos = 'r';
                    }
                }
            }
            telemetry.update();
        }

        while (latch) {//lowers the robot
            if (distanceSensor.getDistance(DistanceUnit.MM) > 120) {
                liftMotor.setPower(-0.25);
            } else {
                latch = false;
                liftMotor.setPower(0);
            }
        }
        liftMotor.setPower(0);

        strafeRight(-0.5);
        sleep(400);
        stopWheels();

        sleep(200);
        goForward(-0.5);
        sleep(100);
        stopWheels();

        sleep(200);

        strafeLeft(-0.5);
        sleep(435);
        stopWheels();
        sleep(500);

        if(cubepos=='l'){
            strafeLeft(-0.5);
            sleep(500);
            stopWheels();
            sleep(100);

            goForward(-0.5);
            sleep(375);
            stopWheels();
            sleep(500);

            strafeLeft(-0.5);
            sleep(900);
            stopWheels();
            sleep(100);

            goForward(-0.5);
            sleep(600);
            stopWheels();
        } else if (cubepos=='c'){
            strafeLeft(-0.5);
            sleep(100);
            stopWheels();
            sleep(200);

            goForward(-0.5);
            sleep(600);
            stopWheels();
            sleep(100);
        } else if (cubepos=='r'){
            strafeRight(-0.5);
            sleep(550);
            stopWheels();
            sleep(100);

            goForward(-0.5);
            sleep(375);
            stopWheels();
            /**sleep(500);

             strafeRight(-0.5);
             sleep(400);
             stopWheels();
             sleep(100);**/


            goForward(-0.5);
            sleep(625);
            stopWheels();
            sleep(100);
        } else {
            strafeLeft(-0.5);
            sleep(35);
            stopWheels();
            sleep(200);

            goForward(-0.5);
            sleep(1000);
            stopWheels();
            sleep(500);
        }

        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


}