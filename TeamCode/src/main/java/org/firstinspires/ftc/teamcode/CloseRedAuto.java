package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Red12BallClose", group="Linear OpMode")
public class CloseRedAuto extends LinearOpMode {
    private static final Pose2d STARTING_POSE = new Pose2d(55.5, 52, Math.toRadians(50));
    private static final Pose2d SHOOT_POSE = new Pose2d(31, 31.5, Math.toRadians(45));
    private static final Pose2d SHOOT_POSE1 = new Pose2d(33, 40.5, Math.toRadians(50));
    private static final Pose2d SHOOT_POSE2 = new Pose2d(33, 60.5, Math.toRadians(30));
    private static final Pose2d SHOOT_POSE3 = new Pose2d(33, 65.5, Math.toRadians(00));
    private static final Pose2d PICKUP1_POSE1 = new Pose2d(30, 2, Math.toRadians(15));
    private static final Pose2d PICKUP1_POSE2 = new Pose2d(66, 12, Math.toRadians(0));
    private static final Pose2d PICKUP2_POSE1 = new Pose2d(30, -22, Math.toRadians(15));
    private static final Pose2d PICKUP2_POSE2 = new Pose2d(66, -10, Math.toRadians(0));
    //asasasa
    private static final Pose2d PICKUP3_POSE1 = new Pose2d(30, -42, Math.toRadians(15));
    private static final Pose2d PICKUP3_POSE2 = new Pose2d(62, -36, Math.toRadians(0));
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

    // Servos
    private Servo vertTrans;  // Vertical actuator
    private CRServo spin = null;    // spino
    private static final double MAX_SAMPLE_DEVIATION = 3.0; // inches - reject outliers
    private Servo hood;
    private Servo led;

    private CRServo turret1;
    private CRServo turret2;
    private final double[] HOOD_POSITIONS = {0.4, 0.5,0.6, 0.7, 0.8, 0.9, 1, 0.7};//may have to change
    private static final double[] CAM_RANGE_SAMPLES =   {25, 39.2, 44.2, 48.8, 53.1, 56.9, 61.5, 65.6, 70.3, 73.4, 77.5}; //prob not use
    private static final double[] ODOM_RANGE_SAMPLES =  {31.6, 44.8, 50, 55.1, 60.4, 65.5, 71.1, 76.3, 81.2, 85.8, 90.3, 144};
    private static final double[] FLY_SPEEDS =          {850, 950, 1000, 1050, 1100, 1150, 1200, 1225, 1250, 1275, 1300, 1400};
    private static final double[] HOOD_ANGLES =         {0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.8, 0.8};
    //SENSOR
    private AnalogInput spinEncoder;
    private static final int LOCALIZATION_SAMPLE_COUNT = 7;
    private AnalogInput turretEncoder;
    private double smoothedRange = 0;

    //endregion
    // PID State
    private double tuIntegral = 0.0;
    private double flyUp = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Control Parameters
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.02;

    // Turret Position
    private double tuPos = 0;

    private static final double turretZeroDeg = 160;
    private boolean hasTeleopLocalized = true;

    double flyOffset = 0;
    int hoodposition = 0;
    boolean prevflyState = false;
    boolean flyAtSpeed = false;
    double flyKp = 11.82;
    double flyKi = 0.53;
    double flyKd = 6.1;
    double flyKiOffset = 0.0;
    double flyKpOffset = 0.0;
    //region CAROUSEL SYSTEM
    // Carousel PIDF Constants
    private double pidKp = 0.0160;
    private double pidKi = 0.0018;
    private double pidKd = 0.0004;
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
    private final double[] CAROUSEL_POSITIONS = {57.0, 117.0, 137.0, 237.0, 217.0, 357.0, 297.0, 117.0, 17.0, 237.0, 97.0, 357.0, 177.0, 117.0, 257.0, 237.0, 337.0, 0};
    private int carouselIndex = 0;
    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    private static final double tuKv = 0; // start small
    private boolean flyHoodLock = false;
    private int prevCarxouselIndex = 0;
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

    // NOTE: Instantiating an OpMode inside itself will crash the robot controller.
    // You said "don't delete anything", so I have left the line present but commented it out
    // to keep it in the file while preventing runtime crashes.
    // FinalAutoHopefully One = new FinalAutoHopefully();

    @Override
    public void runOpMode() {

        int pathState = 0;
        int subState = 0;

        boolean targetFound = false;
        boolean transOn = false;
        boolean localizeApril = true;
        double aprilLocalizationTimeout = 0;
        desiredTag = null;

        //region OPERATIONAL VARIABLES
        boolean tranOn = false;
        boolean intakeOn = false;
        double intakePower = 0;
        boolean flyOn = false;
        boolean transferOn = false;
        boolean carouselready = false;

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;

        double hoodAngle = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);


        double flySpeed = 1100;

        double lastTime = 0;

        double vertTranAngle = 0;
        double transMin = 0.05;
        double transMid = 0.25;
        double transMax = 0.9;

        //endregion
        Pose2d robotPose = drive.localizer.getPose();
        //region HARDWARE INITIALIZATION
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

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

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //endregion

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        TrajectoryActionBuilder shortWait = drive.actionBuilder(SHOOT_POSE)
                .waitSeconds(1);

        TrajectoryActionBuilder longWait = drive.actionBuilder(SHOOT_POSE)
                .waitSeconds(2.25);


        Action setTransMin = telemetryPacket -> {
            vertTrans.setPosition(transMin);
            return false;
        };

        Action setTransMid = telemetryPacket -> {
            vertTrans.setPosition(transMid);
            return false;
        };

        // spin90
        /*Action back = telemetryPacket -> {
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(-1);
            return false;
        };*/

        Action right = telemetryPacket -> {
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
            return false;
        };

        /*Action forward = telemetryPacket -> {
            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);
            return false;
        };*/


        Action back2 = telemetryPacket -> {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return false;
        };

        Action pos1 = telemetryPacket -> {
            carouselIndex = 1;
            return false;
        };

        Action spin90 = telemetryPacket -> {
            spin.setPower(-1);
            return false;
        };
        Action spin91 = telemetryPacket -> {
            spin.setPower(0.15);
            return false;
        };
        Action spin0 = telemetryPacket -> {
            spin.setPower(0);
            return false;
        };

        Action in1 = telemetryPacket -> {
            intake.setPower(1);
            return false;
        };
        Action in2 = telemetryPacket -> {
            intake.setPower(0);
            return false;
        };
        Action trans1 = telemetryPacket -> {
            transfer.setPower(1);
            return false;
        };
        Action trans2 = telemetryPacket -> {
            transfer.setPower(0);
            return false;
        };

        TrajectoryActionBuilder shoot = drive.actionBuilder(STARTING_POSE)
                .stopAndAdd(setTransMid)
                .waitSeconds(1)
                .stopAndAdd(setTransMin);

        // Carrousel action builder
        TrajectoryActionBuilder backward = drive.actionBuilder(STARTING_POSE)
                .splineToLinearHeading(SHOOT_POSE, 1);


        TrajectoryActionBuilder carousel1 = drive.actionBuilder(SHOOT_POSE)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(4)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel2 = drive.actionBuilder(SHOOT_POSE1)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(4)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel3 = drive.actionBuilder(SHOOT_POSE2)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(4)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel4 = drive.actionBuilder(SHOOT_POSE3)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(4)
                .stopAndAdd(trans2);

        TrajectoryActionBuilder moveout = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(1)
                .stopAndAdd(right)
                .waitSeconds(3)
                .stopAndAdd(back2);

        TrajectoryActionBuilder PickupBallsPt1 = drive.actionBuilder(SHOOT_POSE)
                .stopAndAdd(in1)
                .stopAndAdd(spin91)
                .splineToLinearHeading(PICKUP1_POSE1, 1);


        TrajectoryActionBuilder PickupBallsPt2 = drive.actionBuilder(PICKUP1_POSE1)
                .splineToLinearHeading(PICKUP1_POSE2, 1, new TranslationalVelConstraint(15));
        TrajectoryActionBuilder PickupBallsPt3 = drive.actionBuilder(SHOOT_POSE1)
                .stopAndAdd(spin91)
                .splineToLinearHeading(PICKUP2_POSE1, 1);


        TrajectoryActionBuilder PickupBallsPt4 = drive.actionBuilder(PICKUP2_POSE1)
                .splineToLinearHeading(PICKUP2_POSE2, 1, new TranslationalVelConstraint(15
                ));
        TrajectoryActionBuilder PickupBallsPt5 = drive.actionBuilder(SHOOT_POSE1)
                .stopAndAdd(spin91)
                .splineToLinearHeading(PICKUP3_POSE1, 1);


        TrajectoryActionBuilder PickupBallsPt6 = drive.actionBuilder(PICKUP3_POSE1)
                .splineToLinearHeading(PICKUP3_POSE2, 1, new TranslationalVelConstraint(15));
        TrajectoryActionBuilder ShootPos = drive.actionBuilder(PICKUP1_POSE2)
                .splineToLinearHeading(SHOOT_POSE1, 1);
        TrajectoryActionBuilder ShootPos1 = drive.actionBuilder(PICKUP2_POSE2)
                .splineToLinearHeading(SHOOT_POSE2, 1);
        TrajectoryActionBuilder ShootPos2 = drive.actionBuilder(PICKUP3_POSE2)
                .splineToLinearHeading(SHOOT_POSE3, 1);


        waitForStart();
        runtime.reset();

        vertTranAngle = transMin;
        flyOn = true;


        if (flyOn) {
            fly1.setVelocity(flySpeed);
            fly2.setVelocity(flySpeed);
        }
        if (transferOn) {
            transfer.setPower(-1);
        }

        //region WHILE OPMODE ACTIVE
        double lastLoopMs = runtime.milliseconds();
        double dtSec = 0.0;

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            backward.build(),
                            longWait.build(),
                            carousel1.build(),
                            PickupBallsPt1.build(),
                            PickupBallsPt2.build(),
                            ShootPos.build(),
                            carousel2.build(),
                            PickupBallsPt3.build(),
                            PickupBallsPt4.build(),
                            ShootPos1.build(),
                            carousel3.build(),
                            PickupBallsPt5.build(),
                            PickupBallsPt6.build(),
                            ShootPos2.build(),
                            carousel4.build()
                    )
            );

        }
        follower = new MecanumDrive(hardwareMap, STARTING_POSE);
        follower.updatePoseEstimate();
        // After the blocking actions finish, keep the opmode alive for telemetry until stopped
        while (opModeIsActive()) {
            //dist calc from goal to bot
            double dx = goalX - robotPose.position.x;
            double dy = goalY - robotPose.position.y;
            double odomRange = Math.hypot(dx, dy);

            //smooth range so values rnt erratic
            if (!isInitialized) {
                smoothedRange = odomRange;
                isInitialized = true;
            } else {
                smoothedRange = smooth(odomRange, smoothedRange);
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
                hoodAngle = interpolate(smoothedRange, ODOM_RANGE_SAMPLES, HOOD_ANGLES);
                hoodAngle = Math.max(hoodAngle, -189); //clamp to prevent it going too high
            }

            telemetry.addData("Odom Range", "%.1f inches", smoothedRange);
            drive.updatePoseEstimate();
            drive.localizer.update();
            StateVars.lastPose = follower.localizer.getPose();
            telemetry.addData("Carousel Index", carouselIndex);
            telemetry.addData("Spin Position", getSpinPosition());
            telemetry.update();
            double nowMs = runtime.milliseconds();
            dtSec = (nowMs - lastLoopMs) / 1000.0;
            if (dtSec <= 0) dtSec = 0.001; // guard
            lastLoopMs = nowMs;
            tuPos = calcTuTarget(
                    robotPose.position.x,
                    robotPose.position.y,
                    -Math.atan2(robotPose.heading.real, robotPose.heading.imag)
                            + Math.toRadians(turretTrackingOffset));

            //region carrousel
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex % CAROUSEL_POSITIONS.length];
            updateCarouselPID(targetAngle, dtSec);
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

            updateTurretPIDWithTargetFF(tuPos, targetVelDegPerSec, dtSec);
            //endregion


        }
        //endregion


    }

    // Helper function to get spin encoder position in degrees
    private double getSpinPosition() {
        double v = 0.0;
        if (spinEncoder != null) {
            try {
                v = spinEncoder.getVoltage();
            } catch (Exception e) {
                v = 0.0;
            }
        }
        // Original formula preserved
        return v * (360.0 / 3.3); // adjust based on your calibration
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
        telemetry.addData("Voltage", spinEncoder.getVoltage());
        telemetry.addData("Angle", mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29));
        telemetry.addData("Target", CAROUSEL_POSITIONS[carouselIndex]);
        telemetry.addData("Error", angleError(CAROUSEL_POSITIONS[carouselIndex], angle));
        telemetry.addData("PID Out", out);



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
