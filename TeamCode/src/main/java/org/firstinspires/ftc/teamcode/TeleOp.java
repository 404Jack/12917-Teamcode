package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tournament_TeleOp")
public class TeleOp extends LinearOpMode {



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
    public Servo craterArmServo = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public DistanceSensor distanceSensorLeft = null;
    public DistanceSensor distanceSensorRight = null;
    public TouchSensor frontbutton = null;
    public TouchSensor backbutton = null;
    public RevBlinkinLedDriver blinkin = null;
    public boolean ledspam = false;

    public double heading = 0;
    public double sensorHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

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
        frontbutton = hardwareMap.get(TouchSensor.class,"frontbutton");
        backbutton = hardwareMap.get(TouchSensor.class,"backbutton");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"rangeSensor");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeFold.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        sweeperMotor.setDirection(DcMotor.Direction.FORWARD);
        lynchpin.setDirection(DcMotor.Direction.FORWARD);


        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFold.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeFold.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lynchpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFold.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        craterArmServo.setPosition(0);

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();


           blinkin.setPattern((RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES));

           while (opModeIsActive()) {
               // Setup a variable for each drive wheel to save power level for telemetry

               /////////GAMEPAD NUMBER ONE
               double leftPower;
               double rightPower;

               leftPower = gamepad1.right_stick_y;
               rightPower = gamepad1.left_stick_y;
               //Activates sweeper motor to spit out minerals
               if (gamepad1.b) {
                   sweeperMotor.setPower(0.6);
               } else {
                   sweeperMotor.setPower(0);
               }


               if (gamepad1.a) {
                   sweeperMotor.setPower(-1);
               } else {
                   sweeperMotor.setPower(0);
               }

               if (gamepad1.x) {
                   intakeFold.setTargetPosition(1050);
                   intakeFold.setPower(0.4);

               }

               if (gamepad1.start) {
                   intakeFold.setTargetPosition(600);
                   intakeFold.setPower(0.4);
               }

               if (gamepad1.y) {
                   intakeFold.setTargetPosition(160);
                   intakeFold.setPower(1);
               }

           /* if (gamepad1.left_bumper) {
                intakeSlideMotor.setTargetPosition(-700);
                intakeSlideMotor.setPower(-0.4);
            }

           if (gamepad1.right_bumper) {
                intakeSlideMotor.setTargetPosition(0);
                intakeSlideMotor.setPower(0.4);
            }
*/

               //////GAMEPAD NUMBER TWO


               //Brings the lift up all the way
               if (gamepad2.y) {
                   liftMotor.setTargetPosition(7300);
                   liftMotor.setPower(1);
                   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
                   ledspam = false;
               }

               if (gamepad2.start) {
                   liftMotor.setTargetPosition(liftMotor.getCurrentPosition() - 200);
                   liftMotor.setPower(0.4);
                   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
               }

               //brings lift up half-way
               if (gamepad2.x) {
                   liftMotor.setTargetPosition(3700);
                   liftMotor.setPower(1);
                   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
                   ledspam = true;

               }
               //brings lift to the bottom of the slide

               if (gamepad2.a) {
                   liftMotor.setTargetPosition(0);
                   if (ledspam = true) {
                       blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                   }
               }
               //Fail Safe
               if (gamepad2.b) {
                   liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 200);
                   liftMotor.setPower(0.4);
                   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
               } //Fail Safe

               //Activates the sweeper motor to suck in minerals

               if (gamepad2.left_bumper) {
                   leftLiftServo.setPosition(0);
                   rightLiftServo.setPosition(1);
               }

               if (gamepad2.right_bumper) {
                   leftLiftServo.setPosition(0.7);
                   rightLiftServo.setPosition(0.3);
               }

               if (gamepad2.dpad_up) {
                   leftLiftServo.setPosition(0.5);
                   rightLiftServo.setPosition(0.5);
               }

               if (gamepad2.right_stick_button) {
                   leftLiftServo.setPosition(0.1);
                   rightLiftServo.setPosition(0.9);
               }

               if (gamepad2.dpad_down) {
                   lynchpin.setPower(0.5);
               } else {
                   lynchpin.setPower(0);
               }

               leftDrive.setPower(leftPower);
               rightDrive.setPower(rightPower);

           }


    }
}

