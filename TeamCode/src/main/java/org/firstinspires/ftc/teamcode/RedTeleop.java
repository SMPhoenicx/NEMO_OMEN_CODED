package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.TimeUnit;

@TeleOp(name="RedTeleop", group="Linear OpMode")
public class RedTeleop extends LinearOpMode {
    private ElapsedTime pidTimer = new ElapsedTime();
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_TURN = 0.4;
    //region HARDWARE DECLARATIONS
// Drive Motors
    private DcMotor frontLeft = null;
    private boolean isInitialized = false;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Mechanism Motors
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;

    private DcMotor transfer = null;
    private NormalizedColorSensor color = null;

    // Servos
    private Servo vertTrans;  // Vertical actuator
    private CRServo spin = null;    // spino
    private static final double MAX_SAMPLE_DEVIATION = 3.0; // inches - reject outliers
    private Servo hood;
    private Servo led;

    private CRServo turret1;
    private CRServo turret2;
    private char[] colors = {'n', 'n', 'n'};
    private static final double[] CAM_RANGE_SAMPLES =   {25, 39.2, 44.2, 48.8, 53.1, 56.9, 61.5, 65.6, 70.3, 73.4, 77.5}; //prob not use
    private static final double[] ODOM_RANGE_SAMPLES =  {1,1,1,1,1,1,1,1,1,1};
    private static final double[] FLY_SPEEDS =          {1,1,1,1,1,1,1,1,1,1};
    private static final double[] HOOD_AT_DISTANCE = {1,1,1,1,1,1,1,1,1,1};
    private static final double[] FLY_AT_DISTANCE = {1,1,1,1,1,1,1,1,1,1};
    private static final double[][] HOOD_ANGLES =
          {{1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1},
          {1,1,1,1,1}};
    private static final double[][] FLY_MEASURES =
            {{1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1},
            {1,1,1,1,1}};
    //SENSOR
    private AnalogInput spinEncoder;
    private static final int LOCALIZATION_SAMPLE_COUNT = 7;
    private AnalogInput turretEncoder;
    private double smoothedRange = 0;
    private double smoothedFly = 0;

    //endregion
    // PID State
    private double tuIntegral = 0.0;
    private double flyUp = 0.0;
    private double currentIndex = 0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.02;

    // Turret Position
    private double tuPos = 0;

    private static final double turretZeroDeg = 100;
    private boolean hasTeleopLocalized = true;

    double flyOffset = -50;
    double carouseloffset = -4;
    int hoodposition = 0;
    boolean prevflyState = false;
    boolean flyAtSpeed = false;
    double flyKp = 13.82;
    double flyKi = 0.53;
    double flyKd = 10.1;
    double flyKiOffset = 0.0;
    double flyKpOffset = 0.0;
    //region CAROUSEL SYSTEM
    // Carousel PIDF Constants
    private double timer = 0;
    private double timer1 = 0;
    private char currentshot = 'n';
    private double pidKp = 0.0160;
    private double pidKi = 0.0018;
    private double pidKd = 0.0008;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;
    private double localizeTime = 0;
    private double tuKp = 0.0084;
    private double tuKi = 0;
    private double tuKd = 0.0003;
    private double tuKf = 0;

    // Carousel PID State
    private double tuLastTimeMs = 0.0;
    //region LOCALIZATION DEBUG
    private double lastLocalizeRange = 0;
    private double lastLocalizeBearingRaw = 0;
    private double lastLocalizeBearingUsed = 0;
    private double lastLocalizeRobotHeading = 0;
    private double lastLocalizeCalcX = 0;
    private double lastLocalizeCalcY = 0;
    private double lastLocalizeFinalX = 0;
    private double lastLocalizeFinalY = 0;
    private double lastLocalizeTagX = 0;
    private double lastLocalizeTagY = 0;
    private double lastLocalizeGlobalToTag = 0;
    //endregion

    // Carousel Control Parameters
    private final double positionToleranceDeg = 2.0;
    private boolean trackingOn = false;
    private final double outputDeadband = 0.03;

    // Carousel Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private double CAROUSEL_POSITION = 0;
    private int carouselIndex = 0;
    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    private static final double tuKv = 0; // start small
    private boolean flyHoodLock = false;
    private int prevCarxouselIndex = 0;
    private static final Pose2d STARTING_POSE = new Pose2d(0, 0, Math.toRadians(90));
    private List<Pose2d> localizationSamples = new ArrayList<>();
    private double turretTrackingOffset = 93;
    private double lastTurretEncoder = 0;
    private static final double TURRET_TRACKING_GAIN = 0.2;
    private static final double TURRET_DERIVATIVE_GAIN = 0.8;

    //VISION STUFF
    private static final int DESIRED_TAG_ID = 24;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private boolean facingGoal = false;
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;
    private double lastHeadingError = 0;

    private static final double TAG_X_PEDRO = 14.612;
    private static final double TAG_Y_PEDRO = 127.905;
    private static final double ALPHA = 0.8;

    private MecanumDrive follower;
    private static final double TURRET_LIMIT_DEG = 270;
    private Pose2d pose;
    public static MecanumDrive.Params PARAMS = new MecanumDrive.Params();
    private ElapsedTime runtime = new ElapsedTime();
    private static final double goalX = 72;
    private static final double goalY = 72;

    @Override
    public void runOpMode() {

        boolean targetFound = false;
        boolean localizeApril = true;
        double aprilLocalizationTimeout=0;
        desiredTag  = null;
        //initAprilTag();
        //region OPERATIONAL VARIABLES
        // Mechanism States
        boolean TransOn = false;
        boolean TransOn1 = false;
        boolean intakeOn = false;

        double intakePower = 0;
        boolean flyOn = true;
        boolean turretOn = false;

        //Tuning Variables

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;

        double hoodAngle =0;


        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Flywheel Control
        double flySpeed = 1400;

        double lastTime = 0;

        //Transfer
        double vertTranAngle = 0;
        double transMin = 0.05;//when transfers up
        double transMid = 0.25;//when its under intake
        double transMax = 0.85;//shoot

        //endregion

        //region HARDWARE INITIALIZATION
        // Initialize Drive Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");
        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");
        transfer     = hardwareMap.get(DcMotor.class, "transfer");
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        led = hardwareMap.get(Servo.class, "led");
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        //turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        spin.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);

        turret1.setDirection(CRServo.Direction.REVERSE);
        turret2.setDirection(CRServo.Direction.REVERSE);
        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AnalogInput spinAnalog = hardwareMap.get(AnalogInput.class, "espin");

        //endregion

        //setManualExposure(4, 200);  // Use low exposure time to reduce motion blur

        //INIT TELEMETRY
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        follower = new MecanumDrive(hardwareMap, STARTING_POSE);
        follower.localizer.setPose(StateVars.lastPose);
        while (opModeIsActive()) {

            //region DRIVE
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            //endregion

            targetFound = false;
            desiredTag  = null;
            follower.updatePoseEstimate();
            follower.localizer.update();
            Pose2d robotPose = follower.localizer.getPose();

//            //region CAMERA
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                        // Yes, we want to use this tag.
//                        desiredTag = detection;
//                        targetFound = true;
//                    } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                    }
//                }else {
//                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                }
//            }
//
//            // DO WHEN CAMERA TRACKING
//            //MAYBE MAKE THIS WHEN facingGoal BOOL IS TRUE?
//            if (targetFound) {
//                adjustDecimation(desiredTag.ftcPose.range);
//                double range = desiredTag.ftcPose.range;
//
//                if(range <= 13) {
//                    hood.setPosition(HOOD_POSITIONS[0]);
//                }
//                else if (range <= 25){
//                    hood.setPosition(HOOD_POSITIONS[1]);
//                }
//                else if (range <= 38){
//                    hood.setPosition(HOOD_POSITIONS[2]);
//                }
//                else{
//                    hood.setPosition(HOOD_POSITIONS[3]);
//                }
//
//                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//            }
//            //endregion
            if (hasTeleopLocalized) {
                //dist calc from goal to bot
                double dx = goalX - robotPose.position.x;
                double dy = goalY - robotPose.position.y;
                double odomRange = Math.hypot(dx, dy);

                //smooth range so values rnt erratic
                if (!isInitialized) {
                    smoothedRange = odomRange;
                    smoothedFly = fly1.getVelocity();
                    isInitialized = true;
                } else {
                    smoothedRange = smooth(odomRange, smoothedRange);
                    smoothedFly = smooth(fly1.getVelocity(), smoothedFly);
                }

                // increases ki at higher speeds...rework values
                if (smoothedRange > 70) {
                    flyKiOffset = 0.45;
                } else if (smoothedRange < 40) {
                    flyKiOffset = -0.2;
                } else {
                    flyKiOffset = 0.0;
                }

                // interpolate between measured values
                if (!flyHoodLock) {
                    flySpeed = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, FLY_SPEEDS);
                    hoodAngle = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES[0]);
                    hoodAngle = Math.max(hoodAngle, -189); //clamp to prevent it going too high
                }

                telemetry.addData("Odom Range", "%.1f inches", smoothedRange);
            }
            for (int i = 1; i < 10; i++){
                HOOD_AT_DISTANCE[i] = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES[0]);
                FLY_AT_DISTANCE[i] = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, FLY_MEASURES[0]);
            }
            hoodAngle = interpolate(smoothedFly, FLY_AT_DISTANCE, HOOD_AT_DISTANCE);
            //endregion
            //region FLYWHEEL CONTROL
            // manual speed adjust and reset all adjustmentsss
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyUp += 40;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyUp -= 40;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3) {
                flyUp = 0;
            }

            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }
            if (gamepad2.dpadUpWasPressed()){
                hoodposition++;
            }
            hood.setPosition(hoodAngle);
            // Voltage Compensation
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0 / 2450.0;
            double compensatedF = baseF * (13.0 / voltage);
            //set pid, might change this to custom pid to improve loop times, but that also means retuning pid cuz dt
            fly1.setVelocityPIDFCoefficients(flyKp + flyKpOffset, flyKi + flyKiOffset, flyKd, compensatedF);
            fly2.setVelocityPIDFCoefficients(flyKp + flyKpOffset, flyKi + flyKiOffset, flyKd, compensatedF);

            // Set Flywheel Velocity
            if (flyOn) {
                fly1.setVelocity(flySpeed + flyOffset+flyUp);
                fly2.setVelocity(flySpeed + flyOffset+flyUp);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            // check if flywheel is at speed
            double flyTotal = flySpeed + flyOffset+flyUp;
//            flyAtSpeed = (flyTotal - fly1.getVelocity() < FLY_AT_DISTANCE[9]) && (flyTotal - fly1.getVelocity() > FLY_AT_DISTANCE[0]) &&
//                    (flyTotal - fly2.getVelocity() < FLY_AT_DISTANCE[9]) && (flyTotal - fly2.getVelocity() > FLY_AT_DISTANCE[0]);
flyAtSpeed = flyTotal - fly1.getVelocity()<30 && flyTotal - fly1.getVelocity() > -30;
            // ...and update led
            if (!flyOn) {
                led.setPosition(1); // white
            } else if (flyAtSpeed) {
                if (prevflyState != flyAtSpeed) {
                    //TODO TEST IF TS WORKS
                    gamepad1.rumble(300);
                }
                led.setPosition(0.5); // blue
            } else {
                led.setPosition(0.3); // red (ish)
            }
            prevflyState = flyAtSpeed;
            //endregion
            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakePower = 1;
                TransOn1 = !TransOn1;
                intakeOn = !intakeOn;
            }

            // Outtake
            if (gamepad1.leftBumperWasPressed()) {
                intakePower = -0.7;
            }

            if (intakeOn) {
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }
            //endregion

            /*if(gamepad2.dpadDownWasPressed()){
                if(hoodAngle > 0.1){
                    hoodAngle -= 0.1;
                }
            }*/

            /*if(gamepad2.dpadUpWasPressed()){
                if(hoodAngle < 1){
                    hoodAngle += 0.1;
                }
            }*/
            if (currentshot == 'n') {
                if (gamepad2.triangleWasPressed()) {
                    TransOn = !TransOn;
                    if (TransOn) {
                        CAROUSEL_POSITION += 90;
                    } else {
                        CAROUSEL_POSITION -= 90;
                    }
                }
            }
            //hood.setPosition(hoodAngle);
            //region CAROUSEL CONTROL

            //region ADJUST CAROUSEL PID
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <=0.0) dtSec = 1.0/50.0;
            pidLastTimeMs = nowMs;

            // Carousel Navigation
            //Left and Right go to intake positions, aka the odd numbered indices on the pos array

            // ENCODING FOR SERVOS
            double volt = spinAnalog.getVoltage();

            // === PIDF tuning via Gamepad2 ===
            double adjustStepP = 0.1;
            double adjustStepI = 0.200;
            double adjustStepD = 0.1;
            double debounceTime = 175; // milliseconds

            if (runtime.milliseconds() - lastPAdjustTime > debounceTime) {
                if (gamepad1.dpad_right) { flyKp += adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_left) { flyKp -= adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastIAdjustTime > debounceTime) {
                if (gamepad1.dpad_up) { flyKi += adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_down) { flyKi -= adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad2.dpad_up) { flyKd += adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad2.dpad_down) { flyKd -= adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
            }
            /*if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad2.left_bumper) { carouseloffset += 1; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad2.right_bumper) { carouseloffset -= 1; lastDAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastPAdjustTime > debounceTime) {
                if (gamepad1.dpad_right) { pidKp += adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_left) { pidKp -= adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastIAdjustTime > debounceTime) {
                if (gamepad1.dpad_up) { pidKi += adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_down) { pidKi -= adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad2.dpad_up) { pidKd += adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad2.dpad_down) { pidKd -= adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad2.left_bumper) { carouseloffset += 1; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad2.right_bumper) { carouseloffset -= 1; lastDAdjustTime = runtime.milliseconds(); }
            }*/


            // Safety clamp
            pidKp = Math.max(0, pidKp);
            pidKi = Math.max(0, pidKi);
            pidKd = Math.max(0, pidKd);

            // Display PID constants on telemetry
            telemetry.addData("PID Tuning", "Press A/B=P+,P- | X/Y=I+,I- | Dpad Up/Down=D+,D-");
            telemetry.addData("kP", "%.4f", flyKp);
            telemetry.addData("kI", "%.4f", flyKi);
            telemetry.addData("kD", "%.4f", flyKd);

            telemetry.addData("carouseloffset", carouseloffset);
            //endregion

            // always run PID towards the current selected preset while opMode active


            //endregion



            if (!TransOn){
                if (gamepad2.leftBumperWasPressed()) {
                    currentshot = 'p';
                }
                if (gamepad2.rightBumperWasPressed()){
                    currentshot = 'g';
                }
                gamepad2.setLedColor(1,0,0,200);
                transfer.setPower(0);
                if (gamepad2.dpadLeftWasPressed()) {
                    currentIndex -= 1;
                    colors = addX(0, colors, colors[0]);
                    colors = remove(colors, 3);
                    CAROUSEL_POSITION -= 60;
                }
                if (gamepad2.dpadRightWasPressed()) {
                    CAROUSEL_POSITION += 60;
                    colors = addX(3, colors, colors[0]);
                    colors = remove(colors, 0);
                    currentIndex += 1;
                }
                if (currentshot != 'n' && findIndex(colors, currentshot) != -1){
                    TransOn = !TransOn;
                    CAROUSEL_POSITION -= 90;
                    CAROUSEL_POSITION += findIndex(colors, currentshot)*60;
                }
                if (TransOn1){
                  if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 2&&runtime.milliseconds()-timer1>400&&findIndex(colors, 'n') != -1){
                      CAROUSEL_POSITION -= 60;
                      currentIndex += 1;
                      colors[0] = getDetectedColor();
                      colors = addX(3, colors, colors[0]);
                      colors = remove(colors, 0);
                      timer1 = runtime.milliseconds();
                  }
                  transfer.setPower(-1);
                }
                timer = runtime.milliseconds();
            }
            else{

                gamepad2.setLedColor(0,1,0,200);
                gamepad2.rumble(300);
                if (currentshot == 'n') {
                    if (runtime.milliseconds() - timer > 600) {
                        transfer.setPower(1);
                        CAROUSEL_POSITION += 60;
                        colors = addX(3, colors, colors[0]);
                        colors = remove(colors, 0);
                        currentIndex += 1;
                        colors[0] = 'n';
                        timer = runtime.milliseconds();
                    }
                }
                /*else{
                    if (runtime.milliseconds() - timer > 600) {
                        CAROUSEL_POSITION -= findIndex(colors, currentshot)*60;
                        colors[findIndex(colors, currentshot)] = 'n';
                        TransOn = !TransOn;
                        currentshot = 'n';111
                        CAROUSEL_POSITION += 90;
                    }
                }*/


            }
            double targetAngle = (CAROUSEL_POSITION)+carouseloffset;
            carouseloffset = carouseloffset%180;


            //endregion

            //region FLYWHEEL
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed += 60;
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed -= 60;
            }

            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }


            // Voltage Compensation
            //Set custom PID values
            fly1.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);
            fly2.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);

            // Set Flywheel Velocity
            if (flyOn) {
                fly1.setVelocity(flySpeed + flyOffset+flyUp);
                fly2.setVelocity(flySpeed + flyOffset+flyUp);
            }
            else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            /*if (gamepad1.squareWasPressed()) {
                tuPos = turretZeroDeg;
                hasTeleopLocalized = false;
                localizeTime = runtime.milliseconds();
                localizationSamples.clear();
            }*/
            if (gamepad1.triangleWasPressed()) {
                trackingOn = !trackingOn;
                tuIntegral = 0.0;
                tuLastError = 0.0;

                lastTuTargetInit = false;
            }
            if (gamepad1.psWasPressed()){
                follower.localizer.setPose(new Pose2d(-72,-72, Math.toRadians(180)));
            }            //region GOAL TRACKING
            if (trackingOn) {
                if (!hasTeleopLocalized) {
                    tuPos = turretZeroDeg;
                }
                else {
                    tuPos = calcTuTarget(
                            robotPose.position.x,
                            robotPose.position.y,
                            -Math.atan2(robotPose.heading.real, robotPose.heading.imag)
                            + Math.toRadians(turretTrackingOffset));

                }
            }
            //endregion
            if (!trackingOn) {
                //zeros position
                tuPos = normalizeDeg180(turretZeroDeg);
            }
            turn  = -gamepad1.right_stick_x;
//            double adjustStepP = 0.0002;
//            double adjustStepI = 0.0002;
//            double adjustStepD = 0.0001;
//            double debounceTime = 175; // milliseconds

            /*if (runtime.milliseconds() - lastPAdjustTime > debounceTime) {
                if (gamepad1.dpad_right) { tuKp += adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_left) { tuKp -= adjustStepP; lastPAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastIAdjustTime > debounceTime) {
                if (gamepad1.dpad_up) { tuKi += adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_down) { tuKi -= adjustStepI; lastIAdjustTime = runtime.milliseconds(); }
            }
            if (runtime.milliseconds() - lastDAdjustTime > debounceTime) {
                if (gamepad1.dpad_up) { tuKd += adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
                if (gamepad1.dpad_down) { tuKd -= adjustStepD; lastDAdjustTime = runtime.milliseconds(); }
            }




            // Safety clamp
            tuKp = Math.max(0, tuKp);
            tuKi = Math.max(0, tuKi);
            tuKd = Math.max(0, tuKd);

            // Display PID constants on telemetry
            telemetry.addData("PID Tuning", "Press A/B=P+,P- | X/Y=I+,I- | Dpad Up/Down=D+,D-");
            telemetry.addData("kP", "%.4f", tuKp);
            telemetry.addData("kI", "%.4f", tuKi);
            telemetry.addData("kD", "%.4f", tuKd);*/
            //endregion
            //region TURRET CONTROl
            if (!trackingOn) {
                //zeros position
                tuPos = normalizeDeg180(turretZeroDeg);
            }

            double rawTurretTargetDeg = tuPos;
            //wraps position
            double safeTurretTargetDeg = applyTurretLimitWithWrap(rawTurretTargetDeg);
            tuPos = safeTurretTargetDeg;

            double targetVelDegPerSec = 0.0;

            //feedforward
            if (!lastTuTargetInit) {
                lastTuTarget = safeTurretTargetDeg;
                lastTuTargetInit = true;
            } else if (trackingOn) {
                double dTarget = normalizeDeg180(safeTurretTargetDeg - lastTuTarget);
                targetVelDegPerSec = dTarget / Math.max(dtSec, 1e-3);
                lastTuTarget = safeTurretTargetDeg;
            } else {
                // no FF when not tracking
                targetVelDegPerSec = 0.0;
                lastTuTarget = safeTurretTargetDeg;
            }

            updateCarouselPID(targetAngle, dtSec);
            updateTurretPIDWithTargetFF(tuPos, targetVelDegPerSec, dtSec);
            //endregion
            moveRobot(1.5*drive, strafe, -turn);
            moveRobot(1.5*drive, strafe, -turn);

            // ---------- TELEMETRY ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("currentshot: ", currentshot);
            telemetry.addData("colors: ", colors[0]);
            telemetry.addData("colors: ", colors[1]);
            telemetry.addData("colors: ", colors[2]);
            telemetry.addData("x:", robotPose.position.x);
            telemetry.addData("y", robotPose.position.y);
            telemetry.addData("Flywheel Speed Target", "%.0f", flySpeed + flyOffset+flyUp);
            telemetry.addData("CurrentFlyspeed: ", smoothedFly);
            telemetry.addData("Hood Angle", "%.1f°", hood.getPosition());
            telemetry.addData("Carousel Target", "%.1f°", targetAngle);
            telemetry.addData("Turret Angle", "%.1f°", mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29));
            telemetry.addData("Tracking?", trackingOn);
            telemetry.addData("Target Angle: ", tuPos);
            telemetry.addData("Bot Heading Difference to Turret: ", normalizeDeg180(Math.toDegrees(Math.atan2(robotPose.heading.real, robotPose.heading.imag))));
            telemetry.update();
        }
    }
    public static int findIndex(char a[], int t)
    {
        if (a == null)
            return -1;

        int len = a.length;
        int i = 0;

        // traverse in the array
        while (i < len) {

            // if the i-th element is t
            // then return the index
            if (a[i] == t) {
                return i;
            }
            else {
                i = i + 1;
            }
        }

        return -1;
    }
    //region HELPER METHODS
    public static char[] addX(int n, char arr[], char x)
    {

        char newarr[] = new char[n + 1];

        // insert the elements from
        // the old array into the new array
        // insert all elements till n
        // then insert x at n+1
        for (int i = 0; i < n; i++)
            newarr[i] = arr[i];

        newarr[n] = x;

        return newarr;
    }
    public static char[] remove(char[] arr, int in) {

        if (arr == null || in < 0 || in >= arr.length) {
            return arr;
        }

        char[] arr2 = new char[arr.length - 1];

        // Copy the elements except the index
        // from original array to the other array
        for (int i = 0, k = 0; i < arr.length; i++) {
            if (i == in)
                continue;

            arr2[k++] = arr[i];
        }

        return arr2;
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    private void updateTurretPID(double targetAngle, double dt) {
        // read angles 0..360
        double angle = 180-mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29);

        //raw error
        double rawError = -angleError(targetAngle, angle);

        //adds a constant term if it's in a certain direction.
        // we either do this or we change the pid values for each direction.
        // gonna try and see if simpler method works tho
        double compensatedTarget = targetAngle;
        if (rawError < 0) { // moving CCW
            compensatedTarget = (targetAngle) % 360.0;
        }
        // compute shortest signed error [-180,180]
        double error = -angleError(compensatedTarget, angle);

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double d = (error - lastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = tuKp * error +tuKi * integral + tuKd * d;
        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += tuKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }
        telemetry.addData("Turret Target", "%.1f", out);
        turret1.setPower(out);
        turret2.setPower(out);

        tuLastError = error;



    }
    private double smooth(double newValue, double previousValue) {
        return ALPHA * newValue + (1 - ALPHA) * previousValue;
    }
    void updateCarouselPID(double targetAngle, double dt) {
        double ccwOffset = -6.0;
        // read angles 0..360
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);
        //raw error
        double rawError = -angleError(targetAngle, angle);

        //adds a constant term if it's in a certain direction.
        // we either do this or we change the pid values for each direction.
        // gonna try and see if simpler method works tho
        double compensatedTarget = targetAngle;
        if (rawError < 0) { // moving CCW
            compensatedTarget = (targetAngle + ccwOffset) % 360.0;
        }
        // compute shortest signed error [-180,180]
        double error = -angleError(compensatedTarget, angle);

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double d = (error - lastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = pidKp * error + pidKi * integral + pidKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += pidKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        spin.setPower(out);

        // store errors for next derivative calculation
        lastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);


    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        telemetry.addData("Encoder: ", angle);
        return angle;
    }

    // Compute shortest signed difference between two angles
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }
    private boolean applyInitialAprilLocalization(AprilTagDetection tag) {
        if (tag == null) return false;

        // Use Pedro pose for heading
        Pose2d current = follower.localizer.getPose();
        double robotHeading = current.heading.imag;

        double tagX = TAG_X_PEDRO;
        double tagY = TAG_Y_PEDRO;
        double range = tag.ftcPose.range;
        double bearingDeg = tag.ftcPose.bearing;
        double bearingRad = Math.toRadians(bearingDeg);

// Angle from camera to tag in global field frame
        double cameraToTagAngle = robotHeading + bearingRad;

// Camera is 'range' away from tag in the OPPOSITE direction
        double cameraX = tagX - range * Math.cos(cameraToTagAngle);
        double cameraY = tagY - range * Math.sin(cameraToTagAngle);

// Camera offset: 5.5" forward in robot frame
        double cameraOffsetY = 5.5;
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double fieldOffsetX = -cameraOffsetY * sinH;
        double fieldOffsetY = cameraOffsetY * cosH;

// Robot center
        double robotX = cameraX - fieldOffsetX;
        double robotY = cameraY - fieldOffsetY;

        Pose2d candidatePose = new Pose2d(robotX, robotY, robotHeading);

        // Add to samples list
        localizationSamples.add(candidatePose);

        // Need more samples?
        if (localizationSamples.size() < LOCALIZATION_SAMPLE_COUNT) {
            telemetry.addData("Localizing", "Sample %d/%d",
                    localizationSamples.size(), LOCALIZATION_SAMPLE_COUNT);
            return false; // keep collecting
        }

        // We have enough samples - filter outliers and average
        Pose2d averagedPose = filterAndAveragePoses(localizationSamples);

        if (averagedPose != null) {
            follower.localizer.setPose(averagedPose);
            telemetry.addData("Localized!", "x=%.1f y=%.1f h=%.1f",
                    averagedPose.position.x, averagedPose.position.y,
                    Math.toDegrees(averagedPose.heading.imag));

            // Save debug values
            lastLocalizeRange = range;
            lastLocalizeBearingRaw = tag.ftcPose.bearing;
            lastLocalizeBearingUsed = bearingDeg;
            lastLocalizeRobotHeading = Math.toDegrees(robotHeading);
            lastLocalizeCalcX = robotX;
            lastLocalizeCalcY = robotY;
            lastLocalizeFinalX = averagedPose.position.x;
            lastLocalizeFinalY = averagedPose.position.y;
            lastLocalizeTagX = tagX;
            lastLocalizeTagY = tagY;
            // Clear samples for next time
            localizationSamples.clear();
            return true;
        } else {
            telemetry.addData("Localization", "Failed - too much variance, restarting...");
            localizationSamples.clear();
            return false;
        }
    }
    private double interpolate(double x, double[] xValues, double[] yValues) {
        // Clamp to table bounds
        if (x <= xValues[0]) return yValues[0];
        if (x >= xValues[xValues.length - 1]) return yValues[yValues.length - 1];

        // Find surrounding points
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                // Linear interpolation formula
                double t = (x - xValues[i]) / (xValues[i + 1] - xValues[i]);
                return yValues[i] + t * (yValues[i + 1] - yValues[i]);
            }
        }
        return yValues[yValues.length - 1]; // fallback
    }
    private Pose2d filterAndAveragePoses(List<Pose2d> samples) {
        if (samples.isEmpty()) return null;

        // Calculate median position to find center
        List<Double> xVals = new ArrayList<>();
        List<Double> yVals = new ArrayList<>();

        for (Pose2d p : samples) {
            xVals.add(p.position.x);
            yVals.add(p.position.y);
        }

        Collections.sort(xVals);
        Collections.sort(yVals);

        double medianX = xVals.get(xVals.size() / 2);
        double medianY = yVals.get(yVals.size() / 2);

        // Filter out outliers (anything too far from median)
        List<Pose2d> filteredSamples = new ArrayList<>();
        for (Pose2d p : samples) {
            double distFromMedian = Math.hypot(p.position.x - medianX, p.position.y - medianY);
            if (distFromMedian <= MAX_SAMPLE_DEVIATION) {
                filteredSamples.add(p);
            } else {
                telemetry.addData("Rejected Outlier", "dist=%.2f", distFromMedian);
            }
        }

        // Need at least 3 good samples
        if (filteredSamples.size() < 3) {
            return null; // too much variance
        }

        // Average the filtered samples
        double sumX = 0, sumY = 0, sumH = 0;
        for (Pose2d p : filteredSamples) {
            sumX += p.position.x;
            sumY += p.position.y;
            sumH += p.heading.imag;
        }

        int n = filteredSamples.size();
        return new Pose2d(sumX / n, sumY / n, sumH / n);
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(1364.84, 1364.84, 794.707, 525.739)//CAMERA CALLIBRATION VALUES
                .build();

        aprilTag.setDecimation(4);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
    private double calcTuTarget(double robotX, double robotY, double robotHeadingRad) {
        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double headingToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading  = Math.toDegrees(robotHeadingRad);

        //actual turret angle needed
        double turretAngleReal = headingToGoal - robotHeading;

        //converts to angle servos need to turn to to achieve turret angle
        double servoAngle = turretZeroDeg + (((double) 83 /40) * turretAngleReal);

        return normalizeDeg180(servoAngle);
    }
    private double normalizeDeg180(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
    private char getDetectedColor(){
        double dist = ((DistanceSensor) color).getDistance(DistanceUnit.CM);
        telemetry.addData("Distance X", dist);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist1) {
            return 'n';
        }

        NormalizedRGBA colors = color.getNormalizedColors();
        if (colors.alpha == 0) return 'n';
        float nRed = colors.red/colors.alpha;
        float nGreen = colors.green/colors.alpha;
        float nBlue = colors.blue/colors.alpha;

        if(nBlue>nGreen&&nGreen>nRed){//blue green red
            return 'p';
        }
        else if(nGreen>nBlue&&nBlue>nRed&&nGreen>nRed*2){//green blue red
            return 'g';
        }
        return 'n';
    }
    private void updateTurretPIDWithTargetFF(double targetAngle, double targetVelDegPerSec, double dt) {
        double angle = getTurretAngleDeg();

        double error = -angleError(targetAngle, angle);

        tuIntegral += error * dt;
        tuIntegral = clamp(tuIntegral, -tuIntegralLimit, tuIntegralLimit);

        double d = (error - tuLastError) / Math.max(dt, 1e-6);

        double out = -1*tuKp * error + -1*tuKi * tuIntegral + -1*tuKd * d;

        // stiction FF
        if (Math.abs(error) > 1.0) out += tuKf * Math.signum(error);

        // target-rate FF (helps match d(turret)/d(target))
        out += tuKv * targetVelDegPerSec;

        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < tuDeadband) out = 0.0;

        if (Math.abs(error) <= tuToleranceDeg) {
            out = 0.0;
            tuIntegral *= 0.2;
        }

        turret1.setPower(out);
        turret2.setPower(out);

        tuLastError = error;

    }
    private double getTurretAngleDeg() {
        return normalizeDeg180(mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29));
    }
    private double applyTurretLimitWithWrap(double desiredDeg) {
        // Always reason in [-180, 180]
        desiredDeg = normalizeDeg180(desiredDeg);

        // Where the turret actually is right now (also [-180, 180])
        double currentDeg = getTurretAngleDeg()+turretTrackingOffset;

        // Shortest signed rotation from current to desired (e.g. +20, -30, etc.)
        double errorToDesired = normalizeDeg180(desiredDeg - currentDeg);

        // "Ideal" next target if we perfectly matched desired in one step
        double candidateDeg = currentDeg + errorToDesired;

        // Hard safety clamp to keep off the wires
        return clamp(candidateDeg, -TURRET_LIMIT_DEG, TURRET_LIMIT_DEG);
    }
    //endregion
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }

    }

    private void adjustDecimation(double range) {
        int newDecimation;
        if (range > 90) {
            newDecimation = 3;
        } else if (range > 50) {
            newDecimation = 3;
        } else {
            newDecimation = 4;
        }
        aprilTag.setDecimation(newDecimation);
        telemetry.addData("Decimation", "%d", newDecimation);
    }
    //endregion
}
