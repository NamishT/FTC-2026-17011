package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name = "Red BEST Auto Path", group = "Red")
public class RedBESTAuto extends OpMode {


    private Follower follower;
    private Shooter shooter;
    private Intake intake;


    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1Pose = poseFactory.of(6.6715, 8.4861, 180);
    private final Pose point2 = poseFactory.of(59.678, 134.0617, 90);
    private final Pose point2Control1 = poseFactory.of(27.9806, 112.9543, 0);
    private final Pose point3 = poseFactory.of(47.3275, 131.0588, 90);
    private final Pose point3Control1 = poseFactory.of(53.431, 124.5495, 0);
    private final Pose point4 = poseFactory.of(59.1435, 133.9782, 90);
    private final Pose point5 = poseFactory.of(5.7057, 126.0811, 270);
    private final Pose point5Control1 = poseFactory.of(35.3504, 101.7523, 0);
    private final Pose point6 = poseFactory.of(5.3262, 104.9288, 270);
    private final Pose point7 = poseFactory.of(58.8261, 134.1684, 90);
    private final Pose point8 = poseFactory.of(6.7322, 69.3139, 90);
    private final Pose point9 = poseFactory.of(5.4943, 109.291, 90);


    // Pre-built paths (built once in init() instead of inside loop())
    private Path p1, p2, p3, p4, p5, p6, p7, p8, p9;


    public static double VELOCITY_P_TARGET = 1130;
    public static double VELOCITY_N_TARGET = 1100;
    public static double SHOOTER_READY_PERCENT = 0.96;


    public static double SHOOT_TIME_MS = 2200;
    public static double INTAKE_TIME_MS = 1500;
    public static double PATH3_INTAKE_TIME_MS = 1500;
    public static double MAX_PATH_TIME_MS = 7000; // Prevents getting stuck if PID jitters at end of path


    private ElapsedTime actionTimer;
    private ElapsedTime pathTimer;


    private enum AutoState {
        START_SHOOT_SPINUP,
        START_SHOOT_FEED,


        PATH1,
        WAIT_PATH1,
        PATH1_INTAKE,


        PATH2,
        WAIT_PATH2,
        PATH2_SHOOT_SPINUP,
        PATH2_SHOOT_FEED,


        PATH3,
        WAIT_PATH3,
        PATH3_INTAKE,


        PATH4,
        WAIT_PATH4,
        PATH4_SHOOT_SPINUP,
        PATH4_SHOOT_FEED,


        PATH5,
        WAIT_PATH5,


        PATH6,
        WAIT_PATH6,
        PATH6_INTAKE,


        PATH7,
        WAIT_PATH7,
        PATH7_SHOOT_SPINUP,
        PATH7_SHOOT_FEED,


        PATH8,
        WAIT_PATH8,
        PATH9,
        WAIT_PATH9,


        DONE
    }


    private AutoState autoState = AutoState.START_SHOOT_SPINUP;


    private void buildPaths() {
        p1 = Paths.line(start, path1Pose).linear(start, path1Pose);
        p2 = Paths.curve(path1Pose, point2Control1, point2).linear(path1Pose, point2);
        p3 = Paths.curve(point2, point3Control1, point3).linear(point2, point3);
        p4 = Paths.line(point3, point4).linear(point3, point4);
        p5 = Paths.curve(point4, point5Control1, point5).linear(point4, point5);
        p6 = Paths.line(point5, point6).linear(point5, point6);
        p7 = Paths.line(point6, point7).linear(point6, point7);
        p8 = Paths.line(point7, point8).linear(point7, point8);
        p9 = Paths.line(point8, point9).linear(point8, point9);
    }


    private boolean shooterReady() {
        return shooter.getVelocityP() >= VELOCITY_P_TARGET * SHOOTER_READY_PERCENT
                && shooter.getVelocityN() >= VELOCITY_N_TARGET * SHOOTER_READY_PERCENT;
    }


    private void runShooter() {
        // Always use PIDF velocity control so it never fights itself switching modes
        shooter.setVelocityP(VELOCITY_P_TARGET);
        shooter.setVelocityN(VELOCITY_N_TARGET);
    }


    private void startPath(Path path, AutoState nextWaitState) {
        intake.closeFlush();
        intake.setPower(0);
        intake.setPowerGateClose();
        follower.follow(path);
        pathTimer.reset();
        autoState = nextWaitState;
    }


    private boolean isPathFinished() {
        return !follower.isBusy() || pathTimer.milliseconds() >= MAX_PATH_TIME_MS;
    }


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();


        buildPaths();


        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        actionTimer = new ElapsedTime();
        pathTimer = new ElapsedTime();


        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();


        autoState = AutoState.START_SHOOT_SPINUP;


        telemetry.addLine("RED AUTO PATH READY");
        telemetry.update();
    }


    @Override
    public void start() {
        // Spin up immediately on start
        runShooter();
        intake.setPowerGateClose();
        intake.closeFlush();
        autoState = AutoState.START_SHOOT_SPINUP;
    }


    @Override
    public void loop() {
        follower.update();
        OpModeStorage.autonomousEndPose = follower.pose();


        if (autoState != AutoState.DONE) {
            runShooter();
        }


        updateAuto();


        telemetry.addData("Auto State", autoState);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Shooter P", shooter.getVelocityP());
        telemetry.addData("Shooter N", shooter.getVelocityN());
        telemetry.addData("Shooter Ready", shooterReady());
        telemetry.update();
    }


    private void updateAuto() {
        switch (autoState) {


            case START_SHOOT_SPINUP:
                if (shooterReady()) {
                    intake.setPowerGateOpen();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.START_SHOOT_FEED;
                }
                break;


            case START_SHOOT_FEED:
                // Keep gates open and feeding continuously for full SHOOT_TIME_MS
                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH1;
                }
                break;


            case PATH1:
                startPath(p1, AutoState.WAIT_PATH1);
                break;


            case WAIT_PATH1:
                if (isPathFinished()) {
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH1_INTAKE;
                }
                break;


            case PATH1_INTAKE:
                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    autoState = AutoState.PATH2;
                }
                break;


            case PATH2:
                startPath(p2, AutoState.WAIT_PATH2);
                break;


            case WAIT_PATH2:
                if (isPathFinished()) {
                    autoState = AutoState.PATH2_SHOOT_SPINUP;
                }
                break;


            case PATH2_SHOOT_SPINUP:
                if (shooterReady()) {
                    intake.setPowerGateOpen();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH2_SHOOT_FEED;
                }
                break;


            case PATH2_SHOOT_FEED:
                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH3;
                }
                break;


            case PATH3:
                startPath(p3, AutoState.WAIT_PATH3);
                break;


            case WAIT_PATH3:
                if (isPathFinished()) {
                    intake.openFlush();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH3_INTAKE;
                }
                break;


            case PATH3_INTAKE:
                if (actionTimer.milliseconds() >= PATH3_INTAKE_TIME_MS) {
                    intake.closeFlush();
                    intake.setPower(0);
                    autoState = AutoState.PATH4;
                }
                break;


            case PATH4:
                startPath(p4, AutoState.WAIT_PATH4);
                break;


            case WAIT_PATH4:
                if (isPathFinished()) {
                    autoState = AutoState.PATH4_SHOOT_SPINUP;
                }
                break;


            case PATH4_SHOOT_SPINUP:
                if (shooterReady()) {
                    intake.setPowerGateOpen();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH4_SHOOT_FEED;
                }
                break;


            case PATH4_SHOOT_FEED:
                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH5;
                }
                break;


            case PATH5:
                startPath(p5, AutoState.WAIT_PATH5);
                break;


            case WAIT_PATH5:
                if (isPathFinished()) {
                    autoState = AutoState.PATH6;
                }
                break;


            case PATH6:
                startPath(p6, AutoState.WAIT_PATH6);
                break;


            case WAIT_PATH6:
                if (isPathFinished()) {
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH6_INTAKE;
                }
                break;


            case PATH6_INTAKE:
                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    autoState = AutoState.PATH7;
                }
                break;


            case PATH7:
                startPath(p7, AutoState.WAIT_PATH7);
                break;


            case WAIT_PATH7:
                if (isPathFinished()) {
                    autoState = AutoState.PATH7_SHOOT_SPINUP;
                }
                break;


            case PATH7_SHOOT_SPINUP:
                if (shooterReady()) {
                    intake.setPowerGateOpen();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH7_SHOOT_FEED;
                }
                break;


            case PATH7_SHOOT_FEED:
                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH8;
                }
                break;


            case PATH8:
                startPath(p8, AutoState.WAIT_PATH8);
                break;


            case WAIT_PATH8:
                if (isPathFinished()) {
                    autoState = AutoState.PATH9;
                }
                break;


            case PATH9:
                startPath(p9, AutoState.WAIT_PATH9);
                break;


            case WAIT_PATH9:
                if (isPathFinished()) {
                    autoState = AutoState.DONE;
                }
                break;


            case DONE:
                shooter.setPower(0);
                intake.setPower(0);
                intake.setPowerGateClose();
                intake.closeFlush();
                break;
        }
    }


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();
    }
}