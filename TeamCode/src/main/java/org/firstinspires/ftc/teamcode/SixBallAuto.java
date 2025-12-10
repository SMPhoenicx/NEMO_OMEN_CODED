package org.firstinspires.ftc.teamcode;

import android.util.Size;

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

@Autonomous(name="ThreeBallAuto", group="Linear OpMode")
public class SixBallAuto extends LinearOpMode {
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
    private DcMotor transfer1 = null;
    // Servos
    private Servo vertTrans;  // Vertical actuator
    private CRServo spin = null;    // spino
    private Servo hood;

    private CRServo turret1;
    private CRServo turret2;
    private final double[] HOOD_POSITIONS = {0.5,0.65,0.8,1};//may have to change
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

    OnePersonOpMode One = new OnePersonOpMode();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        boolean targetFound = false;
        boolean localizeApril = true;
        double aprilLocalizationTimeout=0;
        desiredTag  = null;
        //region OPERATIONAL VARIABLES
        // Mechanism States
        boolean tranOn = false;
        boolean intakeOn = false;
        double intakePower = 0;
        boolean flyOn = false;
        boolean transferOn = false;

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
        double flySpeed = 1160;

        double lastTime = 0;

        //Transfer
        double vertTranAngle = 0;
        double transMin = 0.05;//when transfers up
        double transMid = 0.25;//when its under intake
        double transMax = 0.9;//shoot

        //endregion
        //region HARDWARE INITIALIZATION
        // Initialize Drive Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");
        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        transfer1       = hardwareMap.get(DcMotorEx.class, "transfer1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        vertTrans = hardwareMap.get(Servo.class, "vtrans");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);
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

        //INIT TELEMETRY
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double timeChange = runtime.milliseconds();

        //initialize carousel values
        double nowMs;
        double dtSec;
        double targetAngle;

        //initial variables
        int pathState = 0;
        int subState = 0;
        int angleIndex = 0;
        int carouselIndex = 0;
        int iter = 0;
        nowMs = runtime.milliseconds();

        while (opModeIsActive()) {
            //case 0: loading the balls
            if (pathState == 0) {
                //robot motion
                if (subState == 0) {
                    if (opModeIsActive() && nowMs < 1000) {
                        moveRobot(0.5, 0.5, 0.5);
                    }

                    if (opModeIsActive() && nowMs < 1500) {
                        moveRobot(0, 0, 0);
                        subState = 1;
                    }
                    nowMs = runtime.milliseconds();
                }
                //loading the ball
                if (subState == 1) {
                    if (nowMs < 1000) {
                        vertTrans.setPosition(transMid);
                        transfer1.setPower(1);
                    }

                    if (nowMs < 1500) {
                        transfer1.setPower(0);
                        hood.setPosition(HOOD_POSITIONS[angleIndex]);
                        subState = 2;
                    }

                    angleIndex++;
                    nowMs = runtime.milliseconds();
                }
                //spinning the carousel
                if (subState == 2) {
                    targetAngle = CAROUSEL_POSITIONS[carouselIndex];
                    updateCarouselPID(targetAngle, nowMs);
                    if (Math.abs(vertTranAngle - targetAngle) < 5) {
                        subState = 3;
                    }
                    nowMs = runtime.milliseconds();
                    carouselIndex++;
                }
                //loop updates
                if (subState == 3) {
                    iter++;
                    subState = 0;
                    if (iter == 3) {
                        pathState = 1;
                        //resetting values
                        angleIndex = 0;
                        carouselIndex = 0;
                        iter = 0;
                    }
                    nowMs = runtime.milliseconds();
                }
            }
            //case 1: shooting the balls
            if (pathState == 1) {
                //moving to the shooting spot
                if (subState == 0) {
                    if (opModeIsActive() && nowMs < 1000) {
                        moveRobot(0.5, 0.5, 0.5);
                    }

                    if (opModeIsActive() && nowMs < 1500) {
                        moveRobot(0, 0, 0);
                        subState = 1;
                    }
                    nowMs = runtime.milliseconds();
                }

                //shooting the ball
                if (subState == 1) {
                    if (nowMs < 1000) {
                        vertTrans.setPosition(transMax);
                        transfer1.setPower(1);
                    }

                    if (nowMs < 1500) {
                        vertTrans.setPosition(transMin);
                        transfer1.setPower(0);
                        hood.setPosition(HOOD_POSITIONS[angleIndex]);
                        subState = 2;
                    }

                    angleIndex++;
                    nowMs = runtime.milliseconds();
                }

                //spinning the carousel
                if (subState == 2) {
                    targetAngle = CAROUSEL_POSITIONS[carouselIndex];
                    updateCarouselPID(targetAngle, nowMs);
                    if (Math.abs(vertTranAngle - targetAngle) < 5) {
                        subState = 3;
                    }
                    nowMs = runtime.milliseconds();
                    carouselIndex++;
                }

                //loop updates
                if (subState == 3) {
                    iter++;
                    subState = 1;
                    if (iter == 3) {
                        pathState = 2; //ending loop for now
                    }
                    nowMs = runtime.milliseconds();
                }
            }
        }
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
}
