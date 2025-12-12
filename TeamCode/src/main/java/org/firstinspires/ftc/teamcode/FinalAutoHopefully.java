package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.roadrunner.ParallelAction;
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
import java.util.concurrent.TimeUnit;
@Autonomous(name = "FinalAutoHopefully")
public class FinalAutoHopefully extends LinearOpMode {
    private static final Pose2d STARTING_POSE = new Pose2d(12, -60, Math.toRadians(90));
    private ElapsedTime pidTimer = new ElapsedTime();
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_TURN = 0.4;
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
    // Servos
    private Servo vertTrans;  // Vertical actuator
    private CRServo spin = null;    // spino
    private Servo hood;

    private CRServo turret1;
    private CRServo turret2;
    private final double[] HOOD_POSITIONS = {0.5, 0.65, 0.8, 1};//may have to change
    //SENSOR
    private AnalogInput spinEncoder;
    private AnalogInput turretEncoder;

    //endregion

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

    private double spindexerAngleDeg = 0.0;
    private double spindexerErrorDeg = 0.0;
    private double spindexerOutput = 0.0;
    private boolean spindexerAtTarget = false;

    private double tuKp = 0;
    private double tuKi = 0;
    private double tuKd = 0.00000;
    private double tuKf = 0.0;

    // Carousel PID State
    private double tuIntegral = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;
    private double tuLastTimeMs = 0.0;

    // Carousel Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;

    // Carousel Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private final double[] CAROUSEL_POSITIONS = {57.0, 177.0, 297.0};
    private int carouselIndex = 0;
    private int prevCarxouselIndex = 0;

    private double turretTrackingOffset = 0;
    private double lastTurretEncoder = 0;
    private static final double TURRET_TRACKING_GAIN = 0.2;
    private static final double TURRET_DERIVATIVE_GAIN = 0.9;

    //VISION STUFF
    private static final int DESIRED_TAG_ID = 20;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private boolean facingGoal = false;
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;
    private double lastHeadingError = 0;

    // NOTE: Instantiating an OpMode inside itself will crash the robot controller.
    // You said "don't delete anything", so I have left the line present but commented it out
    // to keep it in the file while preventing runtime crashes.
    // FinalAutoHopefully One = new FinalAutoHopefully();

    private ElapsedTime runtime = new ElapsedTime();

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

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;

        double hoodAngle = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);


        double flySpeed = 1160;

        double lastTime = 0;

        double vertTranAngle = 0;
        double transMin = 0.05;
        double transMid = 0.25;
        double transMax = 0.9;

        //endregion

        //region HARDWARE INITIALIZATION
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "in");
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        vertTrans = hardwareMap.get(Servo.class, "vtrans");
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

        TrajectoryActionBuilder shortWait = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(1000);

        TrajectoryActionBuilder longWait = drive.actionBuilder(STARTING_POSE)
                .waitSeconds(7000);


        Action setTransMin = telemetryPacket -> {
            vertTrans.setPosition(transMin);
            return false;  // false = action finishes immediately
        };

        Action setTransMid = telemetryPacket -> {
            vertTrans.setPosition(transMid);
            return false;
        };

        // spin90
        Action spin90 = telemetryPacket -> {
            carouselIndex++;
            return false;
        };

        TrajectoryActionBuilder shoot = drive.actionBuilder(STARTING_POSE)
                .stopAndAdd(setTransMin)
                .waitSeconds(1)
                .stopAndAdd(setTransMid);

        // Carrousel action builder
        TrajectoryActionBuilder Carrousel = drive.actionBuilder(STARTING_POSE)
                .stopAndAdd(spin90);



        waitForStart();
        runtime.reset();

        vertTranAngle = transMin;
        flyOn = true;
        intakeOn = true;

        if (flyOn) {
            fly1.setVelocity(flySpeed);
            fly2.setVelocity(flySpeed);
        }
        if (intakeOn) {
            intake.setPower(1);
        }

        //region WHILE OPMODE ACTIVE
        double lastLoopMs = runtime.milliseconds();
        double dtSec = 0.0;

        if (opModeIsActive()) {
            // Build and run actions once
            Actions.runBlocking(
                    new ParallelAction(
                            longWait.build(),
                            shoot.build(),
                            shortWait.build(),
                            Carrousel.build(),
                            shortWait.build(),
                            shoot.build(),
                            shortWait.build(),
                            Carrousel.build(),
                            shortWait.build(),
                            shoot.build()
                    )
            );
        }

        // After the blocking actions finish, keep the opmode alive for telemetry until stopped
        while (opModeIsActive()) {

            double nowMs = runtime.milliseconds();
            dtSec = (nowMs - lastLoopMs) / 1000.0;
            if (dtSec <= 0) dtSec = 0.001; // guard
            lastLoopMs = nowMs;

            //region carrousel
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex % CAROUSEL_POSITIONS.length];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            telemetry.addData("Carousel Index", carouselIndex);
            telemetry.addData("Spin Position", getSpinPosition());
            telemetry.update();

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

    private void updateCarouselPID(double targetAngle, double dt) {
        if (dt <= 0) return;

        double currentAngle = getSpinPosition();
        // basic error (no angle wrapping implemented, but that can be added if you want shortest path)
        double error = targetAngle - currentAngle;

        // Integrator
        integral += error * dt;
        integral = Range.clip(integral, -integralLimit, integralLimit);

        // Derivative
        double derivative = 0.0;
        if (pidLastTimeMs > 0) {
            derivative = (error - lastError) / dt;
        }

        double output = pidKp * error + pidKi * integral + pidKd * derivative + pidKf;
        // scale output to servo power domain; chosen divisor prevents overly large servo commands
        // keep it conservative — if needed tune pidKp/Ki/Kd instead of changing this scaling
        double scaled = output / 180.0; // approximate scale (180 deg -> full power)
        scaled = Range.clip(scaled, -1.0, 1.0);

        if (Math.abs(scaled) < outputDeadband) scaled = 0.0;

        if (spin != null) spin.setPower(scaled);

        lastError = error;
        pidLastTimeMs = runtime.milliseconds();
    }
}
