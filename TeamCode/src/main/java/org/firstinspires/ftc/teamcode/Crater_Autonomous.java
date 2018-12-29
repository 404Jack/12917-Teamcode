package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.VuMarkTarget;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "Crater Side Autonomous" , group="Autonomous")
public class Crater_Autonomous extends LinearOpMode {

    // Detector object
    private GoldAlignDetector detector;
    WebcamName webcamName;
    Dogeforia vuforia;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // Declare OpMode members.
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor intakeSlideMotor = null;
    public DcMotor intakeFold = null;
    public DcMotor sweeperMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor lynchpin = null;
    public Servo leftLiftServo = null;
    public Servo rightLiftServo = null;
    public DistanceSensor distanceSensorLeft = null;
    public DistanceSensor distanceSensorRight = null;
    public Servo craterArmServo = null;

    public double dist1;
    public double dist2;
    public double heading = 0;
    public double sensorHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init_a();

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "intakeSlideMotor");
        sweeperMotor = hardwareMap.get(DcMotor.class, "sweeperMotor");
        intakeFold = hardwareMap.get(DcMotor.class, "intakeFold");
        lynchpin = hardwareMap.get(DcMotor.class, "lynchpin");
        leftLiftServo = hardwareMap.get(Servo.class, "leftLiftServo");
        rightLiftServo = hardwareMap.get(Servo.class, "rightLiftServo");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distSensorRight");
        craterArmServo = hardwareMap.get(Servo.class, "craterArmServo");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);


        //Set all motors to run to position and reset encoders
        SetModeRUN_TO_POSITION();

        SetZeroPowerBrake();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //Get calibration data
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "Put me up");
        telemetry.update();

        waitForStart();

        LowerIntake();

        LowerFromLander();

        ResetLift();

        ResetIntake();

        //LeftGyroTurn(43,0.3);

        boolean aligned = false;

        while (!detector.isFound()){
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 100);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 100);
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
            if (!aligned){
                aligned = detector.getAligned();

            }else{
                break;
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        while (!detector.getAligned() && !aligned){
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 100);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 100);
            leftDrive.setPower(0.02);
            rightDrive.setPower(0.02);

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //GyroTurn(20,0.5);

        DriveForward(3100,1);

        stop();

        //sleep(2000);
        if (detector.getXPosition() == 0) {
            telemetry.addData("Gold Mineral Position", "Left");
            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
            telemetry.update();

            LeftGyroTurn(70,0.2);

            DriveForward(3100,0.9);

            craterArmDeploy();

            stop();


        }
        else if (detector.getXPosition() < 325) {

            telemetry.addData("Gold Mineral Position", "Right");
            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
            telemetry.update();

            RightGyroTurn(-15,0.2);

            DriveForward(3100,0.8);

            craterArmDeploy();

            stop();

        }
        else if (detector.getXPosition() > 325) {
            telemetry.addData("Gold Mineral Position", "Center");
            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
            telemetry.update();


            LeftGyroTurn(43, 0.2);

            CenterMineralAdjustment();

            DriveForward(2700, 0.9);

            craterArmDeploy();

            stop();

        }
    }


    public void init_a() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamName;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 120; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        // detector.enable(); // Start the detector!

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
//    @Override
//    public void loop() {
//        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
//        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
//    }


    public void DriveForward(int distance , double speed){
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void DriveBackwards(int distance, double speed){
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void LeftTurn(int distance,double speed){

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void RightTurn(int distance , double speed){

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }

    //Standard Functions
    public void LowerFromLander(){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lynchpin.setTargetPosition(lynchpin.getCurrentPosition() + 525);
        liftMotor.setPower(-0.8);
        lynchpin.setPower(1);
        while (lynchpin.isBusy()&& opModeIsActive()){}

        //Let robot down
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(.1);
        sleep (1600);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 150);
        liftMotor.setPower(0.4);
        while (liftMotor.isBusy() && opModeIsActive()){}

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 500);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 500);
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
    }
    public void LowerIntake() {
        intakeFold.setTargetPosition(intakeFold.getCurrentPosition() + 400);
        intakeFold.setPower(0.5);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
    }


    public void ResetLift(){
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0.9);
        while (liftMotor.isBusy() && opModeIsActive()){}
    }
    public void DropMarker() {
        leftLiftServo.setPosition(0.7);
        rightLiftServo.setPosition(0.3);
        sleep(1900);
        leftLiftServo.setPosition(0);
        rightLiftServo.setPosition(1);
    }
    public void ResetIntake() {
        intakeFold.setTargetPosition(0);
        intakeFold.setPower(0.4);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
    }
    public void LynchpinReset(){
        lynchpin.setTargetPosition(0);
        lynchpin.setPower(1);
        while (lynchpin.isBusy()&& opModeIsActive()){}
    }
    public void IntakeResetForwardDrive(int distance , double speed) {
        intakeFold.setTargetPosition(0);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        intakeFold.setPower(0.4);
        while (intakeFold.isBusy() & leftDrive.isBusy() & rightDrive.isBusy() && opModeIsActive()) {
        }
    }

    //Gyro Methods
    public void resetGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        sensorHeading = currentHeading;
        heading = 0;
    }
    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double delta = currentHeading - sensorHeading;
        if(delta < -180)
        {
            delta += 360;
        }
        else if(delta >= 180)
        {
            delta -=360;
        }
        heading += delta;
        sensorHeading = currentHeading;
        return heading;
    }


    public void GyroTurn( double degrees, double speed) {
        resetGyro();

        boolean Left = degrees > 0;
        double LeftSpeed = speed;
        double RightSpeed = speed;


        telemetry.addData("heading", heading);
        telemetry.update();
        sleep(1000);
        if (Left){
            LeftSpeed = LeftSpeed * -1;
        }
        else {
            RightSpeed = RightSpeed * -1;
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setPower(LeftSpeed);
        rightDrive.setPower(RightSpeed);
        while (Math.abs(getHeading())< Math.abs(degrees) - 6 && opModeIsActive()) {

            if (Math.abs(degrees)-Math.abs(getHeading()) < 30){
                if (Left) {
                    leftDrive.setPower(-0.35);
                    rightDrive.setPower(0.35);

                }
                else {
                    leftDrive.setPower(0.35);
                    rightDrive.setPower(-0.35);
                }

            }

            telemetry.addData("heading", heading);
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void RightGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -degrees+5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void NegativeRightGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < -degrees+5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void LeftGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < degrees-5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void NegativeLeftGyroTurn(double degrees , double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 8000);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 8000);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > degrees-5 && leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void ResetGyro() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


    }
    public void CenterMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 857;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 857;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void LeftMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 594;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 1120;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void RightMineralAdjustment(){
        double Lcorrect = distanceSensorLeft.getDistance(DistanceUnit.MM) - 1120;
        double Rcorrect = distanceSensorRight.getDistance(DistanceUnit.MM) - 594;
        double preHypotenuse = (Lcorrect * Lcorrect) + (Rcorrect * Rcorrect);
        double hypotenuse = Math.sqrt(preHypotenuse);
        double correctionAngle = Math.acos((hypotenuse * hypotenuse + Lcorrect * Lcorrect - Rcorrect * Rcorrect) / (2 * hypotenuse * Lcorrect));


        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 900);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 900);
        leftDrive.setPower(0.2);
        rightDrive.setPower(-0.2);
        while (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > -correctionAngle + 44 & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            telemetry.addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                }
            });
            telemetry.update();
        }}
    public void craterArmDeploy(){


        craterArmServo.setPosition(0.5);
    }
    //Mode set Protocols
    public void SetModeRUN_TO_POSITION(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lynchpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeFold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lynchpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void SetZeroPowerBrake(){
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void composeTelemetry() {


        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
