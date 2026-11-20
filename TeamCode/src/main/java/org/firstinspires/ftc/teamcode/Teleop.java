package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.blockN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateP;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private FtcDashboard dash;
    private Telemetry myTe;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose redDefaultEndPose = poseFactory.of(5.4943, 109.291, 90);
    private final Pose blueDefaultEndPose = poseFactory.of(136.4809, 39.4076, 270);

    public static double velocityNTarget = 1100;
    public static double velocityPTarget = 1130;
    public static double shooterRecoveryThreshold = 0.98;

    public static double gateNClosed = 0.5;
    public static double gateNOpen = 0.0;

    public static double gatePClosed = 0.5;
    public static double gatePOpen = 0.25;
    public static double blockNClosed = 1.0;
    public static double blockNOpen = -1.0;

    private boolean blockNIsOpen = true;

    public static double intakePower = 1.0;
    public static double triggerThreshold = 0.3;

    public static double redHeadingOffsetDeg = 0.0;
    public static double blueHeadingOffsetDeg = 180.0;

    private boolean isRedAlliance = true;
    private double manualHeadingOffsetRad = 0.0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        Pose startPose = (OpModeStorage.autonomousEndPose != null)
                ? OpModeStorage.autonomousEndPose
                : redDefaultEndPose;
        follower.setPose(startPose);
        follower.update();

        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        blockNIsOpen = true;
        updateBlockN();
        closeGates();
        intake.closeFlush();
        intake.setPower(0);

        myTe.addLine("TeleOp Initialized");
        myTe.addLine("DPAD LEFT = RED | DPAD RIGHT = BLUE");
        myTe.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) {
            isRedAlliance = true;
            if (OpModeStorage.autonomousEndPose == null) {
                follower.setPose(redDefaultEndPose);
            }
        } else if (gamepad1.dpad_right) {
            isRedAlliance = false;
            if (OpModeStorage.autonomousEndPose == null) {
                follower.setPose(blueDefaultEndPose);
            }
        }

        myTe.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        myTe.addData("Pose Source", OpModeStorage.autonomousEndPose != null ? "FROM AUTO" : "DEFAULT END POSE");
        myTe.addData("Start X", follower.pose().x());
        myTe.addData("Start Y", follower.pose().y());
        myTe.addData("Start Heading (deg)", Math.toDegrees(follower.pose().heading()));
        myTe.addLine("DPAD LEFT = RED | DPAD RIGHT = BLUE");
        myTe.update();
    }

    @Override
    public void start() {
        manualHeadingOffsetRad = 0.0;
    }

    @Override
    public void loop() {
        double allianceOffsetRad = Math.toRadians(
                isRedAlliance ? redHeadingOffsetDeg : blueHeadingOffsetDeg
        );

        if (gamepad1.optionsWasPressed()) {
            manualHeadingOffsetRad = follower.pose().heading() - allianceOffsetRad;
        }

        double fieldCentricHeading = follower.pose().heading() - allianceOffsetRad - manualHeadingOffsetRad;

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                turn,
                fieldCentricHeading
        );
        follower.manual(powers);
        follower.update();

        if (gamepad1.triangleWasPressed()) {
            intake.closeFlush();
        }
        if (gamepad1.circleWasPressed()) {
            intake.openFlush();
        }

        shooting_machine(false, false);

        double velocityP = shooter.getVelocityP();
        double velocityN = shooter.getVelocityN();

        boolean pReady = velocityP >= velocityPTarget * shooterRecoveryThreshold;
        boolean nReady = velocityN >= velocityNTarget * shooterRecoveryThreshold;

        myTe.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        myTe.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        myTe.addData("X", follower.pose().x());
        myTe.addData("Y", follower.pose().y());
        myTe.addData("Shooter P Velocity", velocityP);
        myTe.addData("Shooter N Velocity", velocityN);
        myTe.addData("Shooter P ACTUAL Velocity", shooter.getVelocityP());
        myTe.addData("Shooter N Velocity", shooter.getVelocityN());
        myTe.addData("Shooter Ready", pReady && nReady);
        myTe.addData("Block N", blockNIsOpen ? "OPEN" : "CLOSED");
        myTe.update();
    }

    public void shooting_machine(boolean aimRequested, boolean reverseRequested) {
        double velocityP = shooter.getVelocityP();
        double velocityN = shooter.getVelocityN();

        boolean pReady = velocityP >= velocityPTarget * shooterRecoveryThreshold;
        boolean nReady = velocityN >= velocityNTarget * shooterRecoveryThreshold;
        boolean shooterReady = pReady && nReady;

        if (!shooterReady) {
            shooter.setPower(1.0);
        } else {
            shooter.setVelocityP(velocityPTarget);
            shooter.setVelocityN(velocityNTarget);
        }

        boolean shootRequested = gamepad1.right_trigger > triggerThreshold;
        boolean intakeForward = gamepad1.right_bumper;
        boolean intakeReverse = gamepad1.left_trigger > triggerThreshold;

        if (shootRequested && shooterReady) {
            gateN.setPosition(gateNOpen);
            setGatePPosition(gatePOpen);
            intake.setPower(-intakePower);
            return;
        }

        if (intakeReverse) {
            closeGates();
            intake.setPower(intakePower);
            return;
        }

        if (intakeForward) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }

        closeGates();
    }

    private void updateBlockN() {
        if (blockNIsOpen) {
            blockN.setPower(blockNOpen);
        } else {
            blockN.setPower(blockNClosed);
        }
    }

    private void setGatePPosition(double position) {
        double adjustedPosition = position;
        adjustedPosition = Math.max(0.0, Math.min(1.0, adjustedPosition));
        gateP.setPosition(adjustedPosition);
    }

    private void closeGates() {
        gateN.setPosition(gateNClosed);
        setGatePPosition(gatePClosed);
    }

    @Override
    public void stop() {
        follower.manual(0, 0, 0);
        intake.setPower(0);
        intake.closeFlush();
        shooter.stop();
        closeGates();
    }
}