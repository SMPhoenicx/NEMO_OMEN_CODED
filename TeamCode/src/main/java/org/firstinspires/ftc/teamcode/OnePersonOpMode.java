package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="OnePersonOpMode", group="Linear OpMode")
public class OnePersonOpMode extends LinearOpMode {
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
    private final double[] HOOD_POSITIONS = {0.5,0.65,0.8,1};//may have to change
    //SENSOR
    private AnalogInput spinEncoder;

    //endregion

    //region CAROUSEL SYSTEM
    // Carousel PIDF Constants
    private double pidKp = 0.0057;
    private double pidKi = 0.00166;
    private double pidKd = 0.00002;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    // Carousel Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;

    // Carousel Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private final double[] CAROUSEL_POSITIONS = {57.0, 117.0, 177.0, 237.0, 297.0, 357.0};
    private int carouselIndex = 0;
    private int prevCarxouselIndex = 0;



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



    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        boolean targetFound = false;
        boolean localizeApril = true;
        double aprilLocalizationTimeout=0;
        desiredTag  = null;
        initAprilTag();
        //region OPERATIONAL VARIABLES
        // Mechanism States
        boolean tranOn = false;
        boolean intakeOn = false;
        double intakePower = 0;
        boolean flyOn = false;

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
        double transMax = 0.45;//shoot
        
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
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        vertTrans = hardwareMap.get(Servo.class, "vtrans");
        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        turret1 = hardwareMap.get(CRServo.class, "turret1");
        turret2 = hardwareMap.get(CRServo.class, "turret2");
      
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
        //endregion

        setManualExposure(4, 200);  // Use low exposure time to reduce motion blur

        //INIT TELEMETRY
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        vertTrans.setPosition(transMin);

        while (opModeIsActive()) {

            //region DRIVE
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            //endregion

            //region CAMERA
            targetFound = false;
            desiredTag  = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        targetFound = true;
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                }else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // DO WHEN CAMERA TRACKING
            //MAYBE MAKE THIS WHEN facingGoal BOOL IS TRUE?
            if (targetFound) {
                adjustDecimation(desiredTag.ftcPose.range);
                double range = desiredTag.ftcPose.range;

                if(range <= 13) {
                    hood.setPosition(HOOD_POSITIONS[0]);
                }
                else if (range <= 25){
                    hood.setPosition(HOOD_POSITIONS[1]);
                }
                else if (range <= 38){
                    hood.setPosition(HOOD_POSITIONS[2]);
                }
                else{
                    hood.setPosition(HOOD_POSITIONS[3]);
                }

                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }
            //endregion

            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakePower = 1;
                intakeOn = !intakeOn;
            }

            // Outtake
            if (gamepad1.leftBumperWasPressed()) {
                intakePower = -0.6;
            }

            if (intakeOn) {
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }
            //endregion

            //region TRANSFER CONTROL
            if (gamepad2.triangleWasPressed()) {
                if (vertTranAngle == transMax) {
                    vertTranAngle = transMin;
                }else {
                    vertTranAngle = transMax;
                }
            }

            vertTrans.setPosition(vertTranAngle);
            telemetry.addData("VertTrans", vertTranAngle);
            //endregion

            if(gamepad2.dpadDownWasPressed()){
                if(hoodAngle > 0.1){
                    hoodAngle -= 0.1;
                }
            }

            if(gamepad2.dpadUpWasPressed()){
                if(hoodAngle < 1){
                    hoodAngle += 0.1;
                }
            }

            hood.setPosition(hoodAngle);
            //region CAROUSEL CONTROL

            //region ADJUST CAROUSEL PID
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <=0.0) dtSec = 1.0/50.0;
            pidLastTimeMs = nowMs;

            // Carousel Navigation
            //Left and Right go to intake positions, aka the odd numbered indices on the pos array

            if (gamepad2.dpadLeftWasPressed()) {
                if(vertTranAngle == transMin) {
                    carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
                    carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
                }
            }
            if (gamepad2.dpadRightWasPressed()) {
                if(vertTranAngle == transMin) {
                    carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
                    carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
                }
            }

            // Update Carousel PID
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            //region FLYWHEEL
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed += 20;
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed -= 20;
            }

            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }

            // Voltage Compensation
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0 / 2450.0;
            double compensatedF = baseF * (13.0 / voltage);
            //Set custom PID values
            fly1.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);
            fly2.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);

            // Set Flywheel Velocity
            if (flyOn) {
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            if (gamepad1.crossWasPressed()){
                facingGoal = !facingGoal;
            }
//df
            if (facingGoal) {
                if (targetFound) {
                    lastKnownBearing = desiredTag.ftcPose.bearing;
                    lastKnownRange = desiredTag.ftcPose.range;
                    lastDetectionTime = System.currentTimeMillis();

                    double headingError = desiredTag.ftcPose.bearing;

                    double deltaTime = pidTimer.seconds();
                    double derivative = (headingError - lastHeadingError) / deltaTime;
                    pidTimer.reset();

//                    if (Math.abs(headingError) < 2.0) {
//                        turn = 0;
//                    } else {
//                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                    }

                    lastHeadingError = headingError;

                    double turretAngle = lastKnownBearing/360;
                    if(turretAngle < 0){
                        turretAngle = 0.5 - turretAngle;
                    }else if(turretAngle > 0){
                        turretAngle = 0.5 + turretAngle;
                    }
                    turret1.setPower(turretAngle);
                    turret2.setPower(turretAngle);

                    telemetry.addData("Tracking", "LIVE (err: %.1f°, deriv: %.2f)", headingError, derivative);
                }
                else {
                    //TRYING TO PREVENT A LOT OF TRACKING LOSS
                    long timeSinceLost = System.currentTimeMillis() - lastDetectionTime;

                    if (timeSinceLost < PREDICTION_TIMEOUT) {
                        // Continue tracking last known bearing
                        double headingError = lastKnownBearing;

                        double deltaTime = pidTimer.seconds();
                        double derivative = (headingError - lastHeadingError) / deltaTime;
                        pidTimer.reset();

                        if (Math.abs(headingError) < 2.0) {
                            turn = 0;
                        } else {
                            turn = (TURN_P * headingError) + (TURN_D * derivative);
                            turn = Range.clip(turn * -1, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        }

                        lastHeadingError = headingError;

                        telemetry.addData("Tracking", "PREDICTED (lost %dms ago)", timeSinceLost);
                    } else {
                        turn = 0;
                        lastHeadingError = 0;
                        pidTimer.reset();
                        telemetry.addData("Tracking", "LOST");
                    }
                }
            }
            else{
                turn  = -gamepad1.right_stick_x;
                lastHeadingError = 0;                 pidTimer.reset();
            }

            //endregion

            moveRobot(drive, strafe, turn);

            // ---------- TELEMETRY ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel Speed", "%.0f", flySpeed);
            telemetry.addData("Hood Angle", "%.1f°", hood.getPosition());
            telemetry.addData("Carousel Target", "%.1f°", targetAngle);
            telemetry.update();
        }
    }

    //region HELPER METHODS
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

    private void updateCarouselPID(double targetAngle, double dt) {
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
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)//CAMERA CALLIBRATION VALUES
                .build();

        aprilTag.setDecimation(4);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

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
