package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;

import android.net.wifi.p2p.WifiP2pConfig;
import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
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
@Autonomous(name = "RedAutoHopefully")
public class FinalAutoHopefully extends LinearOpMode {
    private static final Pose2d STARTING_POSE = new Pose2d(23, 53, Math.toRadians(50));
    private static final Pose2d SHOOT_POSE = new Pose2d(0, 36, Math.toRadians(42));
    private static final Pose2d SHOOT_POSE1 = new Pose2d(-6, 36, Math.toRadians(50));

    private static final Pose2d SHOOT_POSE2 = new Pose2d(-2, 50, Math.toRadians(25));
    private static final Pose2d SHOOT_POSE3 = new Pose2d(-2, 75, Math.toRadians(10));
    private static final Pose2d PICKUP1_POSE1 = new Pose2d(7, 0, Math.toRadians(15));
    private static final Pose2d PICKUP1_POSE2 = new Pose2d(28, 16, Math.toRadians(0));
    private static final Pose2d PICKUP2_POSE1 = new Pose2d(7, -7, Math.toRadians(15));
    private static final Pose2d PICKUP2_POSE2 = new Pose2d(28, -16, Math.toRadians(0));

    private static final Pose2d PICKUP3_POSE1 = new Pose2d(7, -48, Math.toRadians(15));
    private static final Pose2d PICKUP3_POSE2 = new Pose2d(28, -38, Math.toRadians(0));
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
    private DcMotor transfer = null;
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
    private double pidKp = 0.0260;
    private double pidKi = 0.0018;
    private double pidKd = 0.0002;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    private double carouselAngleDeg = 0.0;
    private double carouselErrorDeg = 0.0;
    private double carouselOutput = 0.0;
    private boolean carouselAtTarget = false;

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
    private int prevCarouselIndex = 0;

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
        boolean carouselready = false;

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;

        double hoodAngle = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);


        double flySpeed = 1400;

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
        transfer = hardwareMap.get(DcMotor.class, "transfer");
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

        TrajectoryActionBuilder shortWait = drive.actionBuilder(SHOOT_POSE)
                .waitSeconds(1);

        TrajectoryActionBuilder longWait = drive.actionBuilder(SHOOT_POSE)
                .waitSeconds(1.5);


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
            intake.setPower(0.8);
            return false;
        };
        Action in2 = telemetryPacket -> {
            intake.setPower(0);
            return false;
        };
        Action trans1 = telemetryPacket -> {
            transfer.setPower(-1);
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
                .waitSeconds(3.5)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel2 = drive.actionBuilder(SHOOT_POSE1)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(3.5)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel3 = drive.actionBuilder(SHOOT_POSE2)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(3.5)
                .stopAndAdd(trans2);
        TrajectoryActionBuilder carousel4 = drive.actionBuilder(SHOOT_POSE3)
                .stopAndAdd(spin91)
                .stopAndAdd(trans1)
                .waitSeconds(3.5)
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

        // After the blocking actions finish, keep the opmode alive for telemetry until stopped
        while (opModeIsActive()) {

            telemetry.addData("Carousel Index", carouselIndex);
            telemetry.addData("Spin Position", getSpinPosition());
            telemetry.update();
            double nowMs = runtime.milliseconds();
            dtSec = (nowMs - lastLoopMs) / 1000.0;
            if (dtSec <= 0) dtSec = 0.001; // guard
            lastLoopMs = nowMs;

            //region carrousel
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex % CAROUSEL_POSITIONS.length];
            updateCarouselPID(targetAngle, dtSec);
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

    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        telemetry.addData("Encoder: ", angle);
        return angle;
    }

}
