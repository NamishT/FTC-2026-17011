package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Blue BEST Auto Path", group = "Blue")
public class BlueBESTAuto extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(84.4965, 134.7604, 90);
    private final Pose path1 = poseFactory.of(134.2181, 134.4604, 0);
    private final Pose point2Start = poseFactory.of(134.2181, 134.4604, 0);
    private final Pose point2 = poseFactory.of(82.9001, 2.828, 270);
    private final Pose point2Control1 = poseFactory.of(113.937, 17.1962, 0);
    private final Pose point2Control2 = poseFactory.of(82.9839, 30.5109, 0);
    private final Pose point3Start = poseFactory.of(82.9001, 2.828, 270);
    private final Pose point3 = poseFactory.of(93.3833, 11.2966, 270);
    private final Pose point3Control1 = poseFactory.of(75.5649, 23.829, 0);
    private final Pose point4Start = poseFactory.of(93.3833, 11.2966, 270);
    private final Pose point4 = poseFactory.of(83.7402, 2.5859, 270);
    private final Pose point4Control1 = poseFactory.of(82.296, 16.6557, 0);
    private final Pose point5Start = poseFactory.of(83.7402, 2.5859, 270);
    private final Pose point5 = poseFactory.of(133.2947, 18.8087, 90);
    private final Pose point5Control1 = poseFactory.of(88.2901, 33.3606, 0);
    private final Pose point6Start = poseFactory.of(133.2947, 18.8087, 90);
    private final Pose point6 = poseFactory.of(136.6817, 38.9779, 90);
    private final Pose point7Start = poseFactory.of(136.6817, 38.9779, 90);
    private final Pose point7 = poseFactory.of(83.0922, 3.6571, 270);
    private final Pose point7Control1 = poseFactory.of(79.4463, 30.3144, 0);
    private final Pose point8Start = poseFactory.of(83.0922, 3.6571, 270);
    private final Pose point8 = poseFactory.of(132.6753, 82.1892, 270);
    private final Pose point9Start = poseFactory.of(132.6753, 82.1892, 270);
    private final Pose point9 = poseFactory.of(136.4809, 39.4076, 270);

    public static double VELOCITY_P_TARGET = 1130;
    public static double VELOCITY_N_TARGET = 1100;
    public static double SHOOTER_READY_PERCENT = 0.98;

    public static double SHOOT_TIME_MS = 2000;
    public static double INTAKE_TIME_MS = 1500;
    public static double PATH3_INTAKE_TIME_MS = 1500;

    private ElapsedTime actionTimer;

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

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }

    private boolean shooterReady() {
        return shooter.getVelocityP() >= VELOCITY_P_TARGET * SHOOTER_READY_PERCENT
                && shooter.getVelocityN() >= VELOCITY_N_TARGET * SHOOTER_READY_PERCENT;
    }

    private void runShooter() {
        if (!shooterReady()) {
            shooter.setPower(1.0);
        } else {
            shooter.setVelocityP(VELOCITY_P_TARGET);
            shooter.setVelocityN(VELOCITY_N_TARGET);
        }
    }

    private void feedWhenReady() {
        if (shooterReady()) {
            intake.setPowerGateOpen();
            intake.setPower(-1.0);
        } else {
            intake.setPowerGateClose();
            intake.setPower(0);
        }
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        actionTimer = new ElapsedTime();

        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();

        autoState = AutoState.START_SHOOT_SPINUP;

        telemetry.addLine("BLUE AUTO PATH READY");
        telemetry.addData("Start X", start.x());
        telemetry.addData("Start Y", start.y());
        telemetry.update();
    }

    @Override
    public void start() {
        autoState = AutoState.START_SHOOT_SPINUP;
    }

    @Override
    public void loop() {
        follower.update();
        OpModeStorage.autonomousEndPose = follower.pose();
        Scheduler.execute();

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
        telemetry.addData("Intake Power", intake.getRightIntakePower());
        telemetry.update();
    }

    private void updateAuto() {
        switch (autoState) {

            case START_SHOOT_SPINUP:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();

                if (shooterReady()) {
                    actionTimer.reset();
                    autoState = AutoState.START_SHOOT_FEED;
                }
                break;

            case START_SHOOT_FEED:
                intake.closeFlush();
                feedWhenReady();

                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH1;
                }
                break;

            case PATH1:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();
                schedule(follow(follower, path1()));
                autoState = AutoState.WAIT_PATH1;
                break;

            case WAIT_PATH1:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    intake.setPowerGateClose();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH1_INTAKE;
                }
                break;

            case PATH1_INTAKE:
                intake.closeFlush();
                intake.setPowerGateClose();
                intake.setPower(-1.0);

                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH2;
                }
                break;

            case PATH2:
                intake.closeFlush();
                intake.setPowerGateClose();
                schedule(follow(follower, path2()));
                autoState = AutoState.WAIT_PATH2;
                break;

            case WAIT_PATH2:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH2_SHOOT_SPINUP;
                }
                break;

            case PATH2_SHOOT_SPINUP:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();

                if (shooterReady()) {
                    actionTimer.reset();
                    autoState = AutoState.PATH2_SHOOT_FEED;
                }
                break;

            case PATH2_SHOOT_FEED:
                intake.closeFlush();
                feedWhenReady();

                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH3;
                }
                break;

            case PATH3:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();
                schedule(follow(follower, path3()));
                autoState = AutoState.WAIT_PATH3;
                break;

            case WAIT_PATH3:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    intake.openFlush();
                    intake.setPowerGateClose();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH3_INTAKE;
                }
                break;

            case PATH3_INTAKE:
                intake.openFlush();
                intake.setPowerGateClose();
                intake.setPower(-1.0);

                if (actionTimer.milliseconds() >= PATH3_INTAKE_TIME_MS) {
                    intake.closeFlush();
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH4;
                }
                break;

            case PATH4:
                intake.closeFlush();
                intake.setPowerGateClose();
                schedule(follow(follower, path4()));
                autoState = AutoState.WAIT_PATH4;
                break;

            case WAIT_PATH4:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH4_SHOOT_SPINUP;
                }
                break;

            case PATH4_SHOOT_SPINUP:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();

                if (shooterReady()) {
                    actionTimer.reset();
                    autoState = AutoState.PATH4_SHOOT_FEED;
                }
                break;

            case PATH4_SHOOT_FEED:
                intake.closeFlush();
                feedWhenReady();

                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH5;
                }
                break;

            case PATH5:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();
                schedule(follow(follower, path5()));
                autoState = AutoState.WAIT_PATH5;
                break;

            case WAIT_PATH5:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH6;
                }
                break;

            case PATH6:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();
                schedule(follow(follower, path6()));
                autoState = AutoState.WAIT_PATH6;
                break;

            case WAIT_PATH6:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    intake.setPowerGateClose();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.PATH6_INTAKE;
                }
                break;

            case PATH6_INTAKE:
                intake.closeFlush();
                intake.setPowerGateClose();
                intake.setPower(-1.0);

                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH7;
                }
                break;

            case PATH7:
                intake.closeFlush();
                intake.setPowerGateClose();
                schedule(follow(follower, path7()));
                autoState = AutoState.WAIT_PATH7;
                break;

            case WAIT_PATH7:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH7_SHOOT_SPINUP;
                }
                break;

            case PATH7_SHOOT_SPINUP:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();

                if (shooterReady()) {
                    actionTimer.reset();
                    autoState = AutoState.PATH7_SHOOT_FEED;
                }
                break;

            case PATH7_SHOOT_FEED:
                intake.closeFlush();
                feedWhenReady();

                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    autoState = AutoState.PATH8;
                }
                break;

            case PATH8:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();
                schedule(follow(follower, path8()));
                autoState = AutoState.WAIT_PATH8;
                break;

            case WAIT_PATH8:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH9;
                }
                break;

            case PATH9:
                intake.closeFlush();
                intake.setPowerGateClose();
                schedule(follow(follower, path9()));
                autoState = AutoState.WAIT_PATH9;
                break;

            case WAIT_PATH9:
                intake.closeFlush();
                intake.setPowerGateClose();
                if (!follower.isBusy()) {
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