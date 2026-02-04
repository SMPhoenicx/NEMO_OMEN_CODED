package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GlobalOffsets;
import org.firstinspires.ftc.teamcode.FlywheelPIDController;
import org.firstinspires.ftc.teamcode.StateVars;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Close Red", group="Robot")
public class CloseRed extends LinearOpMode {
    private ElapsedTime pidTimer = new ElapsedTime();
    private double timeout = 0;
    double TURN_P = 0.06;
    double TURN_D = 0.002;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_TURN = 0.4;
    //Flywheel PID stuff
    private FlywheelPIDController flywheel;
    boolean shootReady = false;

    //region PEDRO VARS
    private Follower follower;
    private Pose startPose, shoot1, movePoint, shoot0, shoot3;
    private Pose[] pickup1 = new Pose[3];
    private Pose[] pickup2 = new Pose[3];
    private Pose[] pickup3 = new Pose[3];
    private Pose[] gatePose = new Pose[3];
    private PathChain scorePath0, scorePath1, scorePath2, scorePath3, moveScore, limelightPath, gatePath, pickupPath1, pickupPath2, pickupPath3;
    //endregion

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
    private RevColorSensorV3 color;
    private CRServo turret1;
    private CRServo turret2;
    private final double[] HOOD_POSITIONS = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.7};//may have to change
    private static final double[] ODOM_RANGE_SAMPLES =  {65.4, 76.5, 86.2, 95.5, 103.5, 110.3, 123.7, 136.9, 149.4, 165, 179.2, 194.8};
    private static final double[] FLY_SPEEDS = {580, 600, 640, 660, 700, 720, 740, 770, 800, 1200, 1250, 1300};
    private static final double[] HOOD_ANGLES=          {.1,.3,.5,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7,.7};
    //SENSOR
    private GoBildaPinpointDriver pinpoint = null;
    private AnalogInput spinEncoder;
    private static final int LOCALIZATION_SAMPLE_COUNT = 7;
    private AnalogInput turretEncoder;
    private double smoothedRange = 0;

    //LIMELIGHT
    private Limelight3A limelight;
    //endregion

    // PID State
    private double tuIntegral = 0.0;
    private double flyUp = 0.0;
    private double tuLastError = 0.0;
    private double tuIntegralLimit = 500.0;

    // Heading PID
    private double lastHeadingError = 0;
    private double KballAngle = 0.67287181376;
    private double Kball = 0.6846;
    private double ballYoffset = 35;
    //limelight path
    private double ballX;
    private double ballY;
    private double ballHeading;
    double limelightWallPos;
    //End Region

    // Control Parameters
    private boolean turretAtTarget = false;
    private final double tuToleranceDeg = 2.0;
    private final double tuDeadband = 0.02;

    //Ball tracking
    double ballTx=0;
    double ballTy=0;

    //Random Booleans

    boolean motifOn = false;
    boolean transOn = false;
    boolean autoShootOn = false;
    boolean intakeLimelightOn = false;
    boolean gateCutoff = false;
    boolean cutoffCarsPID = false;

    // Ball Storage Tracking
    private char[] savedBalls = {'g', 'p', 'p'};
    private boolean[] presentBalls = {false, false, false};
    private int greenPos = 0;

    private boolean CarouselPidArmed = false;
    private boolean CarouselAtTarget = false;
    private boolean intakeOn = false;

    // Turret Position
    private double tuPos = 0;

    private static final double turretZeroDeg = 100;
    private boolean hasTeleopLocalized = true;

    double flyOffset = 0;
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
    private double pidKp = 0.0360;
    private double pidKi = 0.0018;
    private double pidKd = 0.0018;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;
    private double localizeTime = 0;
    private double tuKp = 0.0124;
    private double tuKi = 0.000;
    private double tuKd = 0.0003;
    private double tuKf = 0.001;
    double targetVelDegPerSec = 0;
    double rawTurretTargetDeg = tuPos;
    double safeTurretTargetDeg = applyTurretLimitWithWrap(rawTurretTargetDeg);
    private final PathConstraints shootConstraints = new PathConstraints(0.99, 100, 0.75, 0.8);
    private final PathConstraints gateConstraints = new PathConstraints(0.99, 100, 0.9, 1);

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
    private final double[] CAROUSEL_POSITIONS = {112.5-13, 172.5-13, 232.5-13, 292.5-13, 352-13, 52.5-13};
    private int carouselIndex = 0;
    private double lastFilteredD = 0.0;
    private double lastTuTarget = 0.0;
    private boolean lastTuTargetInit = false;

    private static final double tuKv = 0; // start small
    private boolean flyHoodLock = false;
    private int prevCarouselIndex = 0;
    private double turretTrackingOffset = -12;
    private double lastTurretEncoder = 0;
    private static final double TURRET_TRACKING_GAIN = 0.2;
    private static final double TURRET_DERIVATIVE_GAIN = 0.8;

    //VISION STUFF
    boolean running = true;
    int shootingState = 0;
    private static final int DESIRED_TAG_ID = 24;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag;
    private boolean facingGoal = false;
    private double lastKnownBearing = 0;
    private double lastKnownRange = 0;
    private long lastDetectionTime = 0;
    private static final long PREDICTION_TIMEOUT = 500;

    private static final double TAG_X_PEDRO = 14.612;
    private static final double TAG_Y_PEDRO = 127.905;
    private static final double ALPHA = 0.8;

    private static final double TURRET_LIMIT_DEG = 270;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double goalX = -72;
    private static final double goalY = 72;

    public void createPoses(){
        startPose = new Pose(19.9,123.5,Math.toRadians(54));

        //0 is control point, 1 is endpoint
        pickup1[0] = new Pose(61.82,76.75,Math.toRadians(180));
        pickup1[1] = new Pose(17.5,84,Math.toRadians(180));

        gatePose[0] = new Pose(29.82,77.24,Math.toRadians(90));
        gatePose[1] = new Pose(14.62,75.3,Math.toRadians(90));//14.62 75.3

        pickup2[0] = new Pose(63.97,54.52,Math.toRadians(180));
        pickup2[1] = new Pose(10,58.36,Math.toRadians(180));
        //return from pickup
        pickup2[2] = new Pose(48.083, 54.73,Math.toRadians(180));

        pickup3[0] = new Pose(76.64,30.5,Math.toRadians(180));
        pickup3[1] = new Pose(10,35.58,Math.toRadians(180));

        shoot1 = new Pose(57.5,98.4,Math.toRadians(180));
//        shoot0 = new Pose(60,119,Math.toRadians(150));
        shoot0 = new Pose(54.43,123.77,Math.toRadians(150));
        shoot3 = new Pose(61.32044198895028,116.9171270718232,Math.toRadians(180));
        movePoint = new Pose(31,69.6,Math.toRadians(90));
    }

    public void createPaths(){
        scorePath0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shoot0))
                .setConstraints(shootConstraints)
                .setLinearHeadingInterpolation(startPose.getHeading(),shoot0.getHeading(), 0.5)
                .addParametricCallback(0.75, ()-> {
                    follower.setMaxPower(0.9);
                } )
//                .addParametricCallback(0.87,()-> shootReady=true)
//                .setBrakingStrength(0.6)
                .build();
        pickupPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot0,pickup1[0],pickup1[1]))
                .setLinearHeadingInterpolation(shoot0.getHeading(),pickup1[1].getHeading(),0.2)
                .addParametricCallback(0.42,()->{
                    follower.setMaxPower(0.3);
                    intakeOn = true;
                    pidKp -= 0.002;
                    pidKd += 0.0004;
                })
                .setTimeoutConstraint(500)
                .build();
        pickupPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup2[0],pickup2[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.38,()->{
                    follower.setMaxPower(0.3);
                    intakeOn = true;
                    pidKp -= 0.002;
                    pidKd += 0.0004;
                })
                .setTimeoutConstraint(500)
                .build();
        pickupPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1,pickup3[0],pickup3[1]))
                .setConstantHeadingInterpolation(shoot1.getHeading())
                .addParametricCallback(0.45,()->{
                    follower.setMaxPower(0.3);
                    intakeOn = true;
                    pidKp -= 0.002;
                    pidKd += 0.0004;
                })
                .setTimeoutConstraint(500)
                .build();
        gatePath = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1[1],gatePose[0],gatePose[1]))
                .setConstraints(gateConstraints)
                .setLinearHeadingInterpolation(pickup1[1].getHeading(),gatePose[1].getHeading())
                .build();
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose[1],shoot1))
                .setConstraints(shootConstraints)
                .setLinearHeadingInterpolation(gatePose[1].getHeading(),shoot1.getHeading())
//                .addParametricCallback(0.983,()-> shootReady=true)
                .addParametricCallback(0.984,()-> shootReady=true)
                .build();
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2[1],pickup2[2],shoot1))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot1.getHeading())
//                .addParametricCallback(0.986,()-> shootReady=true)
                .addParametricCallback(0.99,()-> shootReady=true)
                .build();
        scorePath3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3[1],shoot3))
                .setConstraints(shootConstraints)
                .setTranslationalConstraint(1.5)
                .setConstantHeadingInterpolation(shoot3.getHeading())
                .addParametricCallback(0.8, ()-> {
                            follower.setMaxPower(0.85);
                        }
                )
                .addParametricCallback(0.99,()-> shootReady=true)
                .build();
        moveScore = follower.pathBuilder()
                .addPath(new BezierLine(shoot3,movePoint))
                .setLinearHeadingInterpolation(shoot3.getHeading(), movePoint.getHeading())
                .build();
    }

    // NOTE: Instantiating an OpMode inside itself will crash the robot controller.
    // You said "don't delete anything", so I have left the line present but commented it out
    // to keep it in the file while preventing runtime crashes.
    // FinalAutoHopefully One = new FinalAutoHopefully();

    @Override
    public void runOpMode() throws InterruptedException {

        int pathState = 0;
        int subState = 0;

        boolean targetFound = false;
        boolean transOn = false;
        boolean localizeApril = true;
        double aprilLocalizationTimeout = 0;
        desiredTag = null;

        //region OPERATIONAL VARIABLES
        boolean tranOn = false;
        double intakePower = 0;
        boolean flyOn = false;
        boolean transferOn = false;
        boolean carouselready = false;

        double lastPAdjustTime = 0;
        double lastIAdjustTime = 0;
        double lastDAdjustTime = 0;
        double lastFAdjustTime = 0;

        double hoodAngle = 0;
        double hoodOffset = 0;

        double flySpeed = 670;
        int shoot0change = -12;

        double lastTime = 0;

        double vertTranAngle = 0;
        double transMin = 0.05;
        double transMid = 0.25;
        double transMax = 0.9;

        //region HARDWARE INITIALIZATION
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        fly1 = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2 = hardwareMap.get(DcMotorEx.class, "fly2");
        flywheel = new FlywheelPIDController(
                hardwareMap.get(DcMotorEx.class, "fly1"),
                hardwareMap.get(DcMotorEx.class, "fly2")
        );
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

        fly1.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        fly2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        spin.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);

        turret1.setDirection(CRServo.Direction.REVERSE);
        turret2.setDirection(CRServo.Direction.REVERSE);

        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //endregion

        //region CAMERA INIT
        //LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);

//        initAprilTag();
//        setManualExposure(4, 200);
        //endregion

        //region INITIALIZE PEDRO
        createPoses();

        Pose2D ftcStartPose = PoseConverter.poseToPose2D(
                startPose,
                InvertedFTCCoordinates.INSTANCE
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        createPaths();

        StateVars.lastPose = startPose;
        limelightWallPos = pickup1[1].getX();
        //endregion

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();
        runtime.reset();
        pidLastTimeMs = runtime.milliseconds();

        while(opModeIsActive()) {
            follower.update();
            StateVars.lastPose = follower.getPose();

            //region IMPORTANT VARS
            //needed at beginning of loop, don't change location
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            pidLastTimeMs = nowMs;

            if (dtSec <= 0.0) dtSec = 1.0 / 50.0;

            double turnInput = -gamepad1.right_stick_x;
            //endregion

            //region PATH STUFF
            if(!follower.isBusy()&&runtime.milliseconds()>timeout){
                switch(pathState){
                    //region CYCLE ZERO (READ MOTIF)
                    case 0:
                        if(subState==0){
                            follower.followPath(scorePath0,true);
//                            motifOn = true;

                            timeout = runtime.milliseconds()+500;
                            subState++;
                        }
                        //READ MOTIF is subState 1
                        else if(subState==1){
                            tuPos = -78;
                            transfer.setPower(1);
                            spin.setPower(1);
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE ONE
                    case 1:
                        if(subState==0){
                            spin.setPower(0);
                            transfer.setPower(0);
                            follower.followPath(pickupPath1,false);

                            flySpeed += shoot0change;

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            follower.followPath(gatePath,false);
                            tuPos = -78;
                            gateCutoff = true;

                            timeout = runtime.milliseconds()+1400;
                            subState++;
                        }
                        else if(subState==3){
                            gateCutoff = false;
                            follower.followPath(scorePath1,true);
                            transOn = true;
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 4, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE TWO
                    case 2:
                        if(subState==0){
                            follower.followPath(pickupPath2,false);

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            follower.followPath(scorePath2,true);
                            tuPos += 3;
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    //region CYCLE THREE
                    case 3:
                        if(subState==0){
                            follower.followPath(pickupPath3,false);

                            subState++;
                        }
                        //INTAKE is subState 1
                        else if(subState==2){
                            follower.setMaxPower(1);
                            follower.followPath(scorePath3,true);
                            tuPos += 2;
                            autoShootOn = true;
                            shootingState=0;

                            subState++;
                        }
                        //AUTO SHOOTING is subState 3, resets subState, and increments pathState
                        break;
                    //endregion

                    case 4:
//                        transOn=false;
                        follower.followPath(moveScore);
                        pathState++;
//                        flySpeed=0;
                        running=false;
                        break;
                }


            }

/*            //region COLOR SENSOR
            char detectedColor = getRealColor();

            if (CarouselAtTarget && runtime.milliseconds() - lastColorRead > 40) {
                int slot = indexToSlot(carouselIndex);
                if (slot != -1) {
                    savedBalls[slot] = detectedColor;  // 'n', 'g', or 'p'
                    lastColorRead = runtime.milliseconds();
                }
            }
            //endregion

            //region LIMELIGHT VISION PROCESSING
            if(createLimelightPathOn){
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {

                    List <LLResultTypes.DetectorResult> detector = result.getDetectorResults();
                    double maxArea = 0;
                    int maxAreaIndex=0;

                    for (int i=0;i<detector.size();i++) { //checks all results of detector
                        LLResultTypes.DetectorResult detected = detector.get(i);

                        if(detected.getTargetArea()>maxArea) {
                            maxArea=detected.getTargetArea();
                            maxAreaIndex=i;
                        }
                    }
                    LLResultTypes.DetectorResult maxDetected = detector.get(maxAreaIndex);
                    ballTx = maxDetected.getTargetXDegrees();
                    ballTy = maxDetected.getTargetYDegrees()+ballYoffset;

                    telemetry.addData("# of balls detected",detector.size());
                    telemetry.addData("Closest ID",maxDetected.getClassId());
                    telemetry.addData("Closest ball",maxDetected.getClassName());
                    telemetry.addData("Ball offset from camera","X: %.3f Y: %.3f",ballTx,ballTy);
                    pathToBall(ballTx,ballTy);
                    createLimelightPathOn=false;
                } else {
                    telemetry.addData("# of balls detected",0);
                    ballTx=0;
                    ballTy=0;
                }
            }
            //endregion
*/
            //region READ MOTIF
            if(motifOn&&timeout<runtime.milliseconds()){
                int april = readMotif();
                if(april!=-1) {
                    if (april == 21) {
                        greenPos = 0;
                    } else if (april == 22) {
                        greenPos = 1;
                    } else if (april == 23) {
                        greenPos = 2;
                    }
                    motifOn=false;
                    StateVars.patternTagID = april;
                    subState++;
                }
                else if(!follower.isBusy()){
                    motifOn=false;
                    subState++;
                }
            }
            //endregion

            //region INTAKE
            char detectedColor = getRealColor();
            boolean present = isBallPresent();
            int currentSlot = indexToSlot(carouselIndex);
            if(intakeOn&&runtime.milliseconds()>timeout){
                intake.setPower(1);
                transOn=false;
                if (present && savedBalls[currentSlot] == 'n' && CarouselAtTarget) {

                    if (detectedColor != 'n') {
                        savedBalls[currentSlot] = detectedColor;
                        spinClock();
                    }
                }

                if(CarouselFull()||!follower.isBusy()){
                    if(CarouselFull()){
                        intake.setPower(0);
                    }
                    follower.breakFollowing();
                    intakeOn = false;
                    pidKp += 0.0015;

                    subState++;
                }
            }
            //endregion

            //region HOOD CONTROL
            //(angles must be negative for our direction)
            //hood.setPosition((hoodAngle + hoodOffset)/355.0);
            //endregion

            //region CAROUSEL
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            if(!cutoffCarsPID){
                updateCarouselPID(targetAngle+ GlobalOffsets.spindexerOffset, dtSec);
            }
            //endregion

            //region SHOOT PREP
            if(autoShootOn&&shootingState==0){
                int greenIn=-1;
                for(int i=0;i<3;i++){
                    if(savedBalls[i]=='g'){
                        greenIn=i;
                    }
                }
                if(greenIn==-1){
                    for(int i=0;i<3;i++){
                        if(savedBalls[i]=='n' || savedBalls[i]=='b'){
                            greenIn=i;
                        }
                    }
                }
                if(greenIn==-1) greenIn=0;

                int diff = (greenIn + greenPos) % 3;
                if(diff==0) carouselIndex=4;
                else if(diff==1) carouselIndex=0;
                else carouselIndex=2;
                CarouselAtTarget=false;
                timeout = runtime.milliseconds() + 300;

                shootingState++;
            }
            //endregion

            //region AUTO SHOOTING
            //prevent ball not firing
            if(autoShootOn&&shootingState==1&&CarouselAtTarget) transOn = true;

            if(autoShootOn&&runtime.milliseconds()>timeout&&(shootReady||!follower.isBusy())){
                intake.setPower(0);
//                double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;
//                if(shootingState==1&&spindexerAtTarget&&avgSpeed > flySpeed * 0.94 && avgSpeed < flySpeed * 1.08){
                if(shootingState==1){
                    transOn = true;
                    if(turretAtTarget){
                        turret1.setPower(0.86);
                        turret2.setPower(0.86);
                        cutoffCarsPID = true;

                        timeout=runtime.milliseconds()+900;
                        shootingState++;
                    }
                }
                else if(shootingState==2){
                    savedBalls[0]='n'; savedBalls[1]='n'; savedBalls[2]='n';

                    cutoffCarsPID = false;
                    shootReady = false;
                    autoShootOn = false;
                    shootingState++;
                    subState=0;
                    pathState++;
                }
            }
            //endregion1

            //region FLYWHEEL
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            flywheel.updateFlywheelPID(
                    flySpeed,
                    dtSec,
                    voltage
            );

            double avgSpeed = (fly1.getVelocity() + fly2.getVelocity()) / 2.0;

//            if(avgSpeed >= flySpeed){
//                flyKd = 3;
//            }
//            tempServo.setPosition(avgSpeed/(flySpeed*2));
            //endregion

            //region TURRET

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

            //region TRANSFER
            if(transOn){
                transfer.setPower(1);
            }
            else{
                transfer.setPower(0);
            }

            //region GATE CUTOFF
            if(runtime.milliseconds()>timeout&&gateCutoff){
                gateCutoff = false;
                follower.breakFollowing();
            }
            //endregion

            //region TELEMETRY
            if(!running) telemetry.addLine("Done!");

            telemetry.addData("Encoder Fly Speed",avgSpeed);
            telemetry.addData("path state", pathState);
            telemetry.addData("sub state",subState);
            telemetry.addData("Transfer: ", transOn);
            telemetry.addData("shooting state",shootingState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Green Position",greenPos);
            telemetry.addData("actual fly speed","Wheel 1: %.1f Wheel 2: %.1f", fly1.getVelocity(), fly2.getVelocity());
            telemetry.addData("Carousel pos",carouselIndex);
            telemetry.addData("Carousel at target",CarouselAtTarget);
            telemetry.update();
            //endregion
        }
    }
    //region HELPER METHODS
    private char getRealColor(){
        char c1 = getDetectedcolor(color);

        if(c1=='p'){
            return 'p';
        }
        if(c1=='g'){
            return 'g';
        }
        return 'n';
    }

    private char getDetectedcolor(NormalizedColorSensor sensor){
        double dist = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        if (Double.isNaN(dist) || dist > GlobalOffsets.colorSensorDist1) {
            return 'n';
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
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


    private int indexToSlot(int index) {
        switch (index) {
            case 0: return 0;
            case 2: return 1;
            case 4: return 2;
            default: return -1;
        }
    }

    private int slotToIndex(int slot) {
        switch (slot) {
            case 0: return 0;
            case 1: return 2;
            case 2: return 4;
            default: return -1;
        }
    }
    private boolean isBallPresent() {
        double dist1 = ((DistanceSensor) color).getDistance(DistanceUnit.CM);

        NormalizedRGBA colors1 = color.getNormalizedColors();

        boolean s1Detected = !Double.isNaN(dist1) && dist1 < GlobalOffsets.colorSensorDist1;

        if (colors1.alpha == 0) {
            s1Detected = false;
        }

        return s1Detected;
    }
    //region SPINDEXER HELPERS
    public void spinClock() {
        prevCarouselIndex = carouselIndex;
        carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
        carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
    }

    public void spinCounterClock() {
        prevCarouselIndex = carouselIndex;
        carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
        carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
    }

    private void updateCarouselPID(double targetAngle, double dt) {
        if (!CarouselPidArmed) {
            lastError = 0;
            lastFilteredD = 0;
            integral = 0;
            turret1.setPower(0);
            turret2.setPower(0);
            CarouselPidArmed = true;
            return;
        }
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        double targetB = (targetAngle + 180.0) % 360.0;

        double errorA = -angleError(targetAngle, angle);
        double errorB = -angleError(targetB, angle);

        // compute shortest signed error [-180,180]
        double error = (Math.abs(errorA) <= Math.abs(errorB)) ? errorA : errorB;

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double rawD = (error - lastError) / Math.max(dt, 1e-6);

        double d = (lastFilteredD * 0.8) + (rawD * 0.2);
        lastFilteredD = d;

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

        CarouselAtTarget = (Math.abs(error) <= positionToleranceDeg+15);

        turret1.setPower(out);
        turret2.setPower(out);

        lastError = error;

        telemetry.addData("ENCODER VOLTAGE", spinEncoder.getVoltage());
        telemetry.addData("ANGLE", targetAngle);
        telemetry.addData("INTEGRAL", integral);
    }

    public void calculateNearestIndex() {
        double currentAngle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        int bestIndex = carouselIndex;
        double minAbsError = 360.0;

        for (int i = 0; i < CAROUSEL_POSITIONS.length; i++) {
            double baseTarget = CAROUSEL_POSITIONS[i] + GlobalOffsets.spindexerOffset;

            // 1. Check the Normal Target
            double errorNormal = Math.abs(angleError(baseTarget, currentAngle));

            // 2. Check the "Ghost" Target (180 degrees away)
            // Since gearing is 1:2, this is the same physical position
            double ghostTarget = baseTarget + 180.0;
            double errorGhost = Math.abs(angleError(ghostTarget, currentAngle));

            // Find which one is closer
            double localMinError = Math.min(errorNormal, errorGhost);

            // Compare against the global best found so far
            if (localMinError < minAbsError) {
                minAbsError = localMinError;
                bestIndex = i;
            }
        }

        // Update the index.
        // The PID loop will automatically handle choosing between
        // the Normal or Ghost target again in the next frame.
        carouselIndex = bestIndex;
    }
    //endregion

    private void updateTurretPID(double targetAngle, double dt) {
        // read angles 0..360
        double angle = mapVoltageToAngle360(turretEncoder.getVoltage(), 0.01, 3.29);

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
        tuIntegral += error * dt;
        tuIntegral = clamp(tuIntegral, -tuIntegralLimit, tuIntegralLimit);

        // derivative
        double d = (error - tuLastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = tuKp * error + tuKi * tuIntegral + tuKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += tuKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < tuDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= tuToleranceDeg) {
            out = 0.0;
            tuIntegral *= 0.2;
        }

        //to know its set
        turretAtTarget = (Math.abs(error) <= tuToleranceDeg + 5);

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        turret1.setPower(out);
        turret2.setPower(out);

        // store errors for next derivative calculation
        tuLastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Turret Target", "%.1f°", targetAngle);

    }

    private int readMotif(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int numTags=0;
        int lastTagNum = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 21||detection.id == 22||detection.id == 23) {
                    numTags++;
                    lastTagNum=detection.id;
                }
            }
        }
        if(numTags==1) return lastTagNum;
        return -1;
    }
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        return angle;
    }

    // Compute shortest signed difference between two angles
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    private void pathToBall(double tx,double ty){
        double hypotenuse = Math.sqrt((tx*tx) + (ty*ty));
        telemetry.addData("hypotenuse",hypotenuse);
        double angle = Math.atan(tx/(ty-5));

        double distX = (Math.cos(follower.getHeading()-angle)*hypotenuse*Kball);
        double distY = (Math.sin(follower.getHeading()-angle)*hypotenuse*Kball);
        if(ty>50){
            distX = 0;
            distY = 0;
        }else if(hypotenuse>40){
            distX *= 1.7;
            distY *= 1.7;
        }else if(hypotenuse>30){
            distX *= 1.2;
            distY *= 1.2;
        }

        ballX = follower.getPose().getX() + distX;
        ballY = follower.getPose().getY() + distY;

        ballHeading = follower.getHeading()+(-angle*KballAngle);//in radians

        //prevent slamming into wall
        if(ballX<26){//144-118 = 26
            double distXchange = ballX-26;//negative
            double proportion = Math.abs(distXchange/distX);//positive
            double distYchange = distY * proportion;
            ballX -= distXchange;
            ballY -= distYchange;
        }

        Pose ballPose = new Pose(ballX,ballY,ballHeading);

        limelightPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),ballPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),ballPose.getHeading())
                .build();
    }
    private Pose limelightPose(){
        return new Pose();
    }

    private boolean CarouselFull(){
        for(int i=0;i<3;i++){
            if(savedBalls[i]=='n') return false;
        }
        return true;
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
//    private void initAprilTag() {
//
//        Position cameraPosition = new Position(
//                DistanceUnit.INCH,
//                0, //x, right +, left -
//                4, //y, forward +, back -
//                12.5, //z up + down -
//                0
//        );
//
//        YawPitchRollAngles orientation = new YawPitchRollAngles(
//                AngleUnit.DEGREES,
//                0, //yaw, left and right
//                -70, //pitch, forward, back
//                180, //roll, orientation
//                0
//        );
        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//                //.setDrawAxes(false)
//                //.setDrawCubeProjection(false)
//                .setDrawTagOutline(true)
//                .setCameraPose(cameraPosition, orientation)
//                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(904.848699568, 904.848699568, 658.131998572, 340.91602987)
//
//                .build();
//
//        aprilTag.setDecimation(4);
//
//        // Create the vision portal by using a builder.
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .setCameraResolution(new Size(1280, 720))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .build();
//    }

//    private void setManualExposure(int exposureMS, int gain) {
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }

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
        telemetry.addData("Decimation: ", "%d", newDecimation);

    }

}