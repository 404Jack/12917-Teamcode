package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "Crater Side Autonomous 2" , group="Autonomous")
public class Crater_Autonomous_2 extends LinearOpMode {

    // Detector object
    private GoldAlignDetector detector;
    WebcamName webcamName;
    Dogeforia vuforia;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private double TOLERANCE = 0.25;
    private double GAIN = 0.05;

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
    public RevBlinkinLedDriver blinkin = null;

    public double dist1;
    public double dist2;
    public double heading = 0;
    public double sensorHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TurnOnDogeCV();

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
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


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
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

//        LowerIntake();
//
//        LowerFromLander();
//
//        ResetIntake();
//
//        SetModeRUN_TO_POSITION();
//
//        LeftGyroTurn(33, 0.5);
//
//        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//
//        sleep(500);
//        if (detector.isFound()) {
//            if (detector.getXPosition() > 325) {
//                //Center
//                LeftGyroTurn(45, 0.7);
//
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
//
//                DriveForward(1900, 0.8);
//
//                DriveBackwards(1700, 0.8);
//
//                BrakeDrivetrain();
//
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
//
//                LeftGyroTurn(110, 0.8);
//            } else {
//                //Right
//                RightGyroTurn(-11, 0.7);
//
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
//
//                DriveForward(2050, 0.8);
//
//                DriveBackwards(1900, 0.8);
//
//                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
//
//                BrakeDrivetrain();
//
//                LeftGyroTurn(110, 0.8);
//            }
//        } else if (!detector.isFound()) {
//            //Left
//            LeftGyroTurn(73, 0.5);
//
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
//
//            DriveForward(2050, 0.8);
//
//            DriveBackwards(1900, 0.8);
//
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
//
//            BrakeDrivetrain();
//
//            GyroTurn(110, 0.8);
//        }
//
//        DriveForward(1000,0.8);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        DistanceSensorDriveForward();
//
//        while (distanceSensorRight.getDistance(DistanceUnit.INCH) < 6){
//            rightDrive.setPower(0.45);
//            leftDrive.setPower(-0.45);
//        }

        PIDWallFollower();
                                     // driveAlongWall(0.5);
         stop();
    }

    public void TurnOnDogeCV() {
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
        detector.downscale = 0.5; // How much to downscale the input frames

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

    public void DistanceSensorDriveForward() {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (distanceSensorRight.getDistance(DistanceUnit.INCH) > 3) {
            leftDrive.setPower(-0.6);
            rightDrive.setPower(-0.6);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
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
    public void PIDWallFollower(){
           double dist = 5;
           double intergral = 0;
           double error = 0;
           double last_error = 0;
           double derivative = 0;
           double Kd = 0; //3rd
           double Kp = 0.1; //start here Higher is sharper
           double Ki = 0; //2nd
           double finalKp;
           double finalKi;
           double finalKd;
           double steering;

          while (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 10) {
          blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
           error = (dist - distanceSensorRight.getDistance(DistanceUnit.INCH));
           intergral = intergral+error;
           derivative = error - last_error;
           finalKp = Kp * error;
           finalKi =intergral*Ki;
           finalKd  =derivative * Kd;
           steering = finalKp+finalKi+finalKd;
           
           telemetry.addData("steering",steering);
           telemetry.addData("distance",distanceSensorRight.getDistance(DistanceUnit.INCH));
           telemetry.addData("Left power", leftDrive.getPower());
           telemetry.addData("Right power", rightDrive.getPower());
           telemetry.addData("error", error);
              telemetry.update();
               rightDrive.setPower(-0.5 + steering);
               leftDrive.setPower(-0.5 - steering);
               sleep(25);

           last_error = error;
       }
    }

    //Standard Functions
    public void LowerFromLander(){
        lynchpin.setPower(1);
        liftMotor.setPower(-0.8);
        lynchpin.setTargetPosition(525);
        //liftMotor.setTargetPosition(25);
        while (lynchpin.isBusy()&& opModeIsActive()){}

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        //Let robot down
        liftMotor.setTargetPosition(2900);
        liftMotor.setPower(1);
        while (liftMotor.isBusy()&& opModeIsActive()){}

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 400);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 400);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        while (leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 300);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - 300);
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(1);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        while (liftMotor.isBusy() & leftDrive.isBusy() & rightDrive.isBusy() & opModeIsActive()) {}
        sleep(400);
    }
    public void LowerIntake() {
        intakeFold.setTargetPosition(intakeFold.getCurrentPosition() + 400);
        intakeFold.setPower(0.5);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
    }
    public void Sampledrive(int distance, double speed) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - distance);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (leftDrive.isBusy() & rightDrive.isBusy() &opModeIsActive());

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + distance);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + distance);
        ResetIntake();
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (intakeFold.isBusy()& leftDrive.isBusy() & rightDrive.isBusy() &opModeIsActive());
    }
    public void ResetLift(){
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(0.9);
        while (liftMotor.isBusy() && opModeIsActive()){}
    }
    public void DropMarker() {
        intakeFold.setTargetPosition(intakeFold.getCurrentPosition() + 800);
        intakeFold.setPower(0.5);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
        sweeperMotor.setPower(0.6);
        sleep(500);
    }
    public void ResetIntake() {
        intakeFold.setTargetPosition(0);
        intakeFold.setPower(0.4);
        while (intakeFold.isBusy() && opModeIsActive()) {
        }
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
    public void GoldAlign(){
        boolean aligned = false;

        while (!detector.isFound()) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 150);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 150);
            leftDrive.setPower(0.05);
            rightDrive.setPower(0.05);
            if (!aligned) {
                aligned = detector.getAligned();

            } else {
                break;
            }
            sleep(100);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        while (!detector.getAligned() && !aligned) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 75);
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 75);
            leftDrive.setPower(0.03);
            rightDrive.setPower(0.03);

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void DriveToDepot(double speed) {
        SetModePowerDrive();
        while (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 31){

            while (distanceSensorRight.getDistance(DistanceUnit.INCH) > 5) {
                leftDrive.setPower(-speed);
                rightDrive.setPower(-speed + 0.1);
            }
            while (distanceSensorRight.getDistance(DistanceUnit.INCH) < 5) {
                leftDrive.setPower(-speed);
                rightDrive.setPower(-speed + 0.1);
            }
        }
        SetModeRUN_TO_POSITION();

    }
    public void DriveToCrater(double speed) {
        SetModePowerDrive();
        while (distanceSensorLeft.getDistance(DistanceUnit.INCH) > 34){

            if (distanceSensorRight.getDistance(DistanceUnit.INCH) > 5) {
                leftDrive.setPower(speed);
                rightDrive.setPower(speed - 0.1);
            }
            if (distanceSensorRight.getDistance(DistanceUnit.INCH) < 5) {
                leftDrive.setPower(speed);
                rightDrive.setPower(speed - 0.1);
            }
        }
        SetModeRUN_TO_POSITION();

    }
    public void driveAlongWall(double speed) {
        while (distanceSensorLeft.getDistance(DistanceUnit.CM)> 65) {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            double error = distanceSensorRight.getDistance(DistanceUnit.INCH) - 7;
            if (error < TOLERANCE){//The robot is too close to the wall
                leftDrive.setPower(GAIN - speed);
                rightDrive.setPower(-GAIN - speed);
            }
            else if (error > TOLERANCE){//The robot is too far away
                leftDrive.setPower(-GAIN - speed);
                rightDrive.setPower(GAIN - speed);
            } else {
                leftDrive.setPower(-speed);
                rightDrive.setPower(-speed);
            }
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    public void SetModePowerDrive() {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetZeroPowerBrake(){
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
     public void BrakeDrivetrain(){
         leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
