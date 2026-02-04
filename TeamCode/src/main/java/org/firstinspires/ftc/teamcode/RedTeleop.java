package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GlobalOffsets.spindexerOffset;
import static java.lang.Math.round;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="RedTeleop", group="Linear OpMode")
public class RedTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //region HARDWARE DECLARATIONS
    // Drive Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Mechanism Motors
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    private DcMotor transfer = null;

    // Servos
    private Servo hood;
    private Servo led;
    private CRServo turret1;
    private CRServo turret2;
    private CRServo spin = null;

    private NormalizedColorSensor color = null;
    private AnalogInput turretEncoder;
    private AnalogInput spinEncoder;
    private GoBildaPinpointDriver pinpoint = null;
    //endregion

    //region SHOOTING SYSTEM
    private double flyTargetSpeed = 0.0;
    private static final double[] CAM_RANGE_SAMPLES =   {25, 39.2, 44.2, 48.8, 53.1, 56.9, 61.5, 65.6, 70.3, 73.4, 77.5}; //prob not use
    private static final double[] ODOM_RANGE_SAMPLES =  {65.4, 76.5, 86.2, 95.5, 103.5, 110.3, 123.7, 136.9, 149.4, 165, 179.2, 194.8};
    private static final double[] FLY_SPEEDS =          {580, 600, 640, 660, 700, 720, 740, 770, 800, 1200, 1250, 1300};
    private static final double[] HOOD_ANGLES=          {.1,.3,.5,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7};

    private boolean flyHoodLock = false;
    private double smoothedRange = 0;
    //endregion

    //region WTF
    //SENSOR
    private static final int LOCALIZATION_SAMPLE_COUNT = 7;
    private static final double MAX_SAMPLE_DEVIATION = 3.0; // inches - reject outliers

    private double currentIndex = 0;

    private double timer = 0;
    private double timer1 = 0;
    private char currentshot = 'n';

    // 57, 177, and 297 face the intake; others face the transfer
    private static final Pose STARTING_POSE = new Pose(0, 0, Math.toRadians(90));
    private List<Pose> localizationSamples = new ArrayList<>();
    private double turretTrackingOffset = -12;

    private static final double ALPHA = 0.8;
    //endregion

    //region HOOD SYSTEM
    private double hoodAngle = 0;
    private double hoodOffset = 0;
    //endregion

    //region FLYWHEEL SYSTEM
    private FlywheelPIDController flywheel;
    double flyKp = 13.82;
    double flyKi = 0.53;
    double flyKd = 10.1;

    boolean prevflyState = false;
    boolean flyAtSpeed = false;
    double flyKiOffset = 0.0;
    double flyKpOffset = 0.0;
    //endregion

    //region SPINDEXER SYSTEM
    // Spindexer PIDF Constants
    private double pidKp = 0.0360;
    private double pidKi = 0.0018;
    private double pidKd = 0.0018;
    private double pidKf = 0.000;

    // Spindexer PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    // Spindexer Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;
    private boolean spindexerOverride = false;
    private double overrideTime = 0.0;

    // Spindexer Positions
    private double spindexeroffset = -80;
    private double SPINDEXER_POSITION = 0;
    private final double[] SPINDEXER_POSITIONS = {112.5-13, 172.5-13, 232.5-13, 292.5-13, 352.5-13, 52.50-13};
    private int spindexerIndex = 0;
    private int prevSpindexerIndex = 0;
    private int greenPos = 0;

    // Ball Storage Tracking
    private char[] colors = {'n', 'n', 'n'};
    private boolean[] presentBalls = {false, false, false};
    //endregion

    //region TURRET SYSTEM
    // PIDF Constants
    private double tuKp = 0.0124;
    private double tuKi = 0.0;
    private double tuKd = 0.0003;
    private double tuKf = 0.0;
    private static final double tuKv = 0.00;

    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    // PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.02;

    // Turret Position
    private double tuPos = 0.0;

    private static final double turretZeroDeg = 100;
    private static final double TURRET_LIMIT_DEG = 135.0;
    private double tuOffset = 0.0;
    //endregion

    //region NAVIGATION & LOCALIZATION
    private Follower follower;
    private boolean trackingOn = false;
    private boolean hasManuallyLocalized = true;
    private boolean isInitialized = false;
    private double localizeTime = 0;
    //endregion

    //region VARIANT VARS (Alliance Specific)
    private static final double goalX = 143;
    private static final double goalY = 163;
    private static final int DESIRED_TAG_ID = 24; //blue=20, red=24
    private static final Pose LOCALIZE_POSE = new Pose(135, 8.9, Math.toRadians(0));
    private static final double TAG_X_PEDRO = 14.612;
    private static final double TAG_Y_PEDRO = 127.905;
    //endregion

    @Override
    public void runOpMode() {

        //region OPERATIONAL VARIABLES
        // Mechanism States
        boolean transOn = false;
        boolean intakeOn = false;
        boolean flyOn = false;
        boolean flyAtSpeed = false;
        boolean prevflyState = false;

        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Flywheel Control
        double flySpeed = 1400;
        double flyOffset = 0;
        double lastTime = 0;

        double intakePower = 0;
        //Tuning Variables

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;
        //endregion

        //region HARDWARE INITIALIZATION
        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");
        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");
        transfer     = hardwareMap.get(DcMotor.class, "transfer");

        // Servos
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        led = hardwareMap.get(Servo.class, "led");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");

        // Sensors
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        spin.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);

        turret1.setDirection(CRServo.Direction.REVERSE);
        turret2.setDirection(CRServo.Direction.REVERSE);
        flywheel = new FlywheelPIDController(
                hardwareMap.get(DcMotorEx.class, "fly1"),
                hardwareMap.get(DcMotorEx.class, "fly2")
        );
        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //endregion

        //region PRE-START
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StateVars.lastPose);

        telemetry.addData("Initialized", "Ready to go!");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //endregion

        while (opModeIsActive()) {
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <=0.0) dtSec = 1.0/50.0;
            pidLastTimeMs = nowMs;

            //region PEDRO
            follower.update();
            Pose robotPose = follower.getPose();
            //endregion

            //region AUTO FLYSPEED/ANGLE
            double dx = goalX - robotPose.getX();
            double dy = goalY - robotPose.getY();
            double odomRange = Math.hypot(dx, dy);

            //smooth range so values rnt erratic
            if (!isInitialized) {
                smoothedRange = odomRange;
                isInitialized = true;
            } else {
                smoothedRange = smooth(odomRange, smoothedRange);
            }

            // can remove this with new system
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
                hoodAngle = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES);
                hoodAngle = Math.max(hoodAngle, -189); //clamp to prevent it going too high
            }

            telemetry.addData("Odom Range", "%.1f inches", smoothedRange);
            //endregion

            //region FLYWHEEL CONTROL
            // manual speed adjust and reset all adjustmentsss
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset += 40;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flyOffset -= 40;
                lastTime = runtime.milliseconds();
            }
            if (gamepad2.left_trigger > 0.3 && gamepad2.right_trigger > 0.3) {
                flyOffset = 0;
                hoodOffset = 0;
            }
            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }
            if (gamepad2.squareWasPressed()) {
                spindexeroffset -= 4;
            }
            if (gamepad2.crossWasPressed()) {
                spindexeroffset += 4;
            }
            // Voltage Compensation
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0 / 2450.0;
            double compensatedF = baseF * (13.0 / voltage);
            if (flyOn) {
                flywheel.updateFlywheelPID(
                        flySpeed + flyOffset,
                        dtSec,
                        voltage
                );
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }


            // check if flywheel is at speed
            double measuredVelocity = fly2.getVelocity();
            double flyTotal = flySpeed + flyOffset;
            flyAtSpeed = Math.abs(flyTotal - measuredVelocity) < 30;

            // ...and update led
            if (!flyOn) {
                led.setPosition(1); // white
            } else if (flyAtSpeed) {
                if (prevflyState != flyAtSpeed) {
                    gamepad1.rumble(300);
                }
                led.setPosition(0.5); // blue
            } else {
                led.setPosition(0.3); // red (ish)
            }
            prevflyState = flyAtSpeed;
            //endregion

            //region HOOD CONTROL
            //position adjustments
            if (gamepad2.dpadUpWasPressed()) {
                hoodOffset += 0.05;
            }
            if (gamepad2.dpadDownWasPressed()) {
                hoodOffset -= 0.05;
            }

            //pid
            hood.setPosition(hoodAngle + hoodOffset);
            //endregion

            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakePower = 1;
                intakeOn = !intakeOn;
                //transOn = false
                // idk if u want this here
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

            //region SPINDEXER STUFF
            if (currentshot == 'n') {
                if (gamepad2.triangleWasPressed()) {
                    transOn = !transOn;
                    if (transOn) {
                        SPINDEXER_POSITION += 90;
                    } else {
                        SPINDEXER_POSITION -= 90;
                    }
                }
            }

            double volt = spinEncoder.getVoltage();
            if (gamepad2.leftBumperWasPressed()){
                turretTrackingOffset -= 2;
            }
            if (gamepad2.rightBumperWasPressed()){
                turretTrackingOffset += 2;
            }
            //bro what...
            if (!transOn){
//                if (gamepad2.leftBumperWasPressed()) {
//                    currentshot = 'p';
//                }
//                if (gamepad2.rightBumperWasPressed()){
//                    currentshot = 'g';
//                }
                gamepad2.setLedColor(1,0,0,200);
                transfer.setPower(0);
                if (gamepad2.dpadLeftWasPressed()) {
                    currentIndex -= 1;
                    colors = addX(3, colors, colors[2]);
                    colors = remove(colors, 3);
                    SPINDEXER_POSITION -= 60;
                }
                if (gamepad2.dpadRightWasPressed()) {
                    SPINDEXER_POSITION += 60;
                    colors = addX(3, colors, colors[0]);
                    colors = remove(colors, 0);
                    currentIndex += 1;
                }
                if (currentshot != 'n' && findIndex(colors, currentshot) != -1){
                    transOn = !transOn;
                    SPINDEXER_POSITION -= 90;
                    SPINDEXER_POSITION += findIndex(colors, currentshot)*60;
                }
                if (intakeOn){
                    if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 2&&runtime.milliseconds()-timer1>500&&findIndex(colors, 'n') != -1){
                        SPINDEXER_POSITION -= 60;
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
                    if (runtime.milliseconds() - timer > 120) {
                        transfer.setPower(1);
                        SPINDEXER_POSITION += 10;
                        colors = addX(3, colors, colors[0]);
                        colors = remove(colors, 0);
                        currentIndex += 1;
                        colors[0] = 'n';
                        timer = runtime.milliseconds();
                    }
                }
            }
            double targetAngle = (SPINDEXER_POSITION)+spindexeroffset;
            spindexeroffset = spindexeroffset%180; //what

            updateSpindexerPID(targetAngle, dtSec);
            //endregion

            //region TURRET
            if (gamepad1.triangleWasPressed()) {
                trackingOn = !trackingOn;
                tuIntegral = 0.0;
                tuLastError = 0.0;

                lastTuTargetInit = false;
            }
            if (gamepad1.psWasPressed()){
                follower.setPose(new Pose(9,9, Math.toRadians(180)));
            }            //region GOAL TRACKING
            if (trackingOn) {
                    tuPos = calcTuTarget(
                            robotPose.getX(),
                            robotPose.getY(),
                            robotPose.getHeading()
                                    + Math.toRadians(turretTrackingOffset));

            }
            else {
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
            updateTurretPIDWithTargetFF(tuPos, targetVelDegPerSec, dtSec);
            //endregion

            //region TUNING TO REMOVE AT SOME POINT (i hope)
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

            // Safety clamp
            pidKp = Math.max(0, pidKp);
            pidKi = Math.max(0, pidKi);
            pidKd = Math.max(0, pidKd);

            telemetry.addData("PID Tuning", "Press A/B=P+,P- | X/Y=I+,I- | Dpad Up/Down=D+,D-");
            telemetry.addData("kP", "%.4f", flyKp);
            telemetry.addData("kI", "%.4f", flyKi);
            telemetry.addData("kD", "%.4f", flyKd);
            //endregion
            //endregion

            //region DRIVE
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            moveRobot(1.5*drive, strafe, -turn);
            //endregion

            //region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("currentshot: ", currentshot);
            telemetry.addData("colors: ", colors[0]);
            telemetry.addData("colors: ", colors[1]);
            telemetry.addData("colors: ", colors[2]);
            telemetry.addData("x:", robotPose.getX());
            telemetry.addData("y", robotPose.getY());
            telemetry.addData("Flywheel Speed Target", "%.0f", flySpeed);
            telemetry.addData("Flymeasure: ", measuredVelocity);
            telemetry.addData("Hood Angle", "%.1f°", hood.getPosition());
            telemetry.addData("Hood Target: ", hoodAngle + hoodOffset);
            telemetry.addData("Spindexer Target", "%.1f°", targetAngle);
            telemetry.addData("Turret Angle", "%.1f°", mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29));
            telemetry.addData("Tracking?", trackingOn);
            telemetry.addData("Target Angle: ", tuPos);
            telemetry.addData("Bot Heading Difference to Turret: ", normalizeDeg180(Math.toDegrees(robotPose.getHeading())));
            telemetry.update();
            //endregion
        }
    }

    //region ALL METHODS
    //region METHODS
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
    public static char[] addX(int n, char arr[], char x)
    {

        char newarr[] = new char[arr.length + 1];

        // insert the elements from
        // the old array into the new array
        // insert all elements till n
        // then insert x at n+1
        for (int i = 0; i < n - 1; i++)
            newarr[i] = arr[i];
        for (int i = n; i< arr.length + 1; i++){
            newarr[i] = arr[i-1];
        }

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

    void updateSpindexerPID(double targetAngle, double dt) {
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
        telemetry.addData("Spindexer Target", "%.1f°", targetAngle);


    }
    //endregion

    //region HELPER METHODS
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        telemetry.addData("Encoder: ", angle);
        return angle;
    }
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
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
    private Pose filterAndAveragePoses(List<Pose> samples) {
        if (samples.isEmpty()) return null;

        // Calculate median position to find center
        List<Double> xVals = new ArrayList<>();
        List<Double> yVals = new ArrayList<>();

        for (Pose p : samples) {
            xVals.add(p.getX());
            yVals.add(p.getY());
        }

        Collections.sort(xVals);
        Collections.sort(yVals);

        double medianX = xVals.get(xVals.size() / 2);
        double medianY = yVals.get(yVals.size() / 2);

        // Filter out outliers (anything too far from median)
        List<Pose> filteredSamples = new ArrayList<>();
        for (Pose p : samples) {
            double distFromMedian = Math.hypot(p.getX() - medianX, p.getY() - medianY);
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
        for (Pose p : filteredSamples) {
            sumX += p.getX();
            sumY += p.getY();
            sumH += p.getHeading();
        }

        int n = filteredSamples.size();
        return new Pose(sumX / n, sumY / n, sumH / n);
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
    private double smooth(double newValue, double previousValue) {
        return ALPHA * newValue + (1 - ALPHA) * previousValue;
    }
    //endregion

    //region TURRET METHODS
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

    //region CAMERA METHODS
//    private boolean applyInitialAprilLocalization(AprilTagDetection tag) {
//        if (tag == null) return false;
//
//        // Use Pedro pose for heading
//        Pose current = follower.getPose();
//        double robotHeading = current.getHeading();
//
//        double tagX = TAG_X_PEDRO;
//        double tagY = TAG_Y_PEDRO;
//        double range = tag.ftcPose.range;
//        double bearingDeg = tag.ftcPose.bearing;
//        double bearingRad = Math.toRadians(bearingDeg);
//
//// Angle from camera to tag in global field frame
//        double cameraToTagAngle = robotHeading + bearingRad;
//
//// Camera is 'range' away from tag in the OPPOSITE direction
//        double cameraX = tagX - range * Math.cos(cameraToTagAngle);
//        double cameraY = tagY - range * Math.sin(cameraToTagAngle);
//
//// Camera offset: 5.5" forward in robot frame
//        double cameraOffsetY = 5.5;
//        double cosH = Math.cos(robotHeading);
//        double sinH = Math.sin(robotHeading);
//
//        double fieldOffsetX = -cameraOffsetY * sinH;
//        double fieldOffsetY = cameraOffsetY * cosH;
//
//// Robot center
//        double robotX = cameraX - fieldOffsetX;
//        double robotY = cameraY - fieldOffsetY;
//
//        Pose candidatePose = new Pose(robotX, robotY, robotHeading);
//
//        // Add to samples list
//        localizationSamples.add(candidatePose);
//
//        // Need more samples?
//        if (localizationSamples.size() < LOCALIZATION_SAMPLE_COUNT) {
//            telemetry.addData("Localizing", "Sample %d/%d",
//                    localizationSamples.size(), LOCALIZATION_SAMPLE_COUNT);
//            return false; // keep collecting
//        }
//
//        // We have enough samples - filter outliers and average
//        Pose averagedPose = filterAndAveragePoses(localizationSamples);
//
//        if (averagedPose != null) {
//            follower.setPose(averagedPose);
//            telemetry.addData("Localized!", "x=%.1f y=%.1f h=%.1f",
//                    averagedPose.getX(), averagedPose.getY(),
//                    Math.toDegrees(averagedPose.getHeading()));
//
//            // Save debug values
//            lastLocalizeRange = range;
//            lastLocalizeBearingRaw = tag.ftcPose.bearing;
//            lastLocalizeBearingUsed = bearingDeg;
//            lastLocalizeRobotHeading = Math.toDegrees(robotHeading);
//            lastLocalizeCalcX = robotX;
//            lastLocalizeCalcY = robotY;
//            lastLocalizeFinalX = averagedPose.getX();
//            lastLocalizeFinalY = averagedPose.getY();
//            lastLocalizeTagX = tagX;
//            lastLocalizeTagY = tagY;
//            // Clear samples for next time
//            localizationSamples.clear();
//            return true;
//        } else {
//            telemetry.addData("Localization", "Failed - too much variance, restarting...");
//            localizationSamples.clear();
//            return false;
//        }
//    }
//    private void setManualExposure(int exposureMS, int gain) {
//        if (visionPortal == null) return;
//
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting for stream...");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        if (!isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//
//    }
//
//    private void adjustDecimation(double range) {
//        int newDecimation;
//        if (range > 90) {
//            newDecimation = 3;
//        } else if (range > 50) {
//            newDecimation = 3;
//        } else {
//            newDecimation = 4;
//        }
//        aprilTag.setDecimation(newDecimation);
//        telemetry.addData("Decimation", "%d", newDecimation);
//    }

    //    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawTagOutline(true)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(1364.84, 1364.84, 794.707, 525.739)//CAMERA CALLIBRATION VALUES
//                .build();
//
//        aprilTag.setDecimation(4);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .setCameraResolution(new Size(640, 480))
////                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .build();
//    }
    //endregion
    //endregion
}