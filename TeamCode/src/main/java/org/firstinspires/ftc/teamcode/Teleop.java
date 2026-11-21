package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.subsystems.Intake.blockN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateP;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@TeleOp(name = "Teleop")
public class Teleop extends OpMode {


    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;


    private FtcDashboard dash;
    private Telemetry myTe;


    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose redDefaultEndPose = poseFactory.of(9.5, 109.3, 270);
    private final Pose blueDefaultEndPose = poseFactory.of(136.4809, 39.4076, 270);


    public static double velocityNTarget = 1100;
    public static double velocityPTarget = 1130;
    public static double shooterRecoveryThreshold = 0.92;


    public static double gateNClosed = 0.5;
    public static double gateNOpen = 0.0;


    public static double gatePClosed = 0.5;
    public static double gatePOpen = 0.25;
    public static double blockNClosed = 1.0;
    public static double blockNOpen = -1.0;


    private boolean blockNIsOpen = true;


    public static double intakePower = 1.0;
    public static double triggerThreshold = 0.3;


    public static double redHeadingOffsetDeg = 180.0;
    public static double blueHeadingOffsetDeg = 0.0;


    private boolean isRedAlliance = true;
    private double manualHeadingOffsetRad = 0.0;


    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);


        Pose startPose = (OpModeStorage.autonomousEndPose != null)
                ? OpModeStorage.autonomousEndPose
                : redDefaultEndPose;


        drivebase.setFieldStartPose(
                startPose.x(),
                startPose.y(),
                Math.toDegrees(startPose.heading())
        );


        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);


        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);


        blockNIsOpen = true;
        updateBlockN();
        closeGates();
        intake.closeFlush();
        intake.setPower(0);


        myTe.addLine("TeleOp Initialized (Pinpoint Field-Centric)");
        myTe.addLine("DPAD LEFT = BLUE | DPAD RIGHT = RED");
        myTe.update();
    }


    @Override
    public void init_loop() {
        drivebase.updatePinpoint();


        if (gamepad1.dpad_left && !isRedAlliance) {
            isRedAlliance = true;
            if (OpModeStorage.autonomousEndPose == null) {
                drivebase.setFieldStartPose(
                        redDefaultEndPose.x(),
                        redDefaultEndPose.y(),
                        Math.toDegrees(redDefaultEndPose.heading())
                );
            }
        } else if (gamepad1.dpad_right && isRedAlliance) {
            isRedAlliance = false;
            if (OpModeStorage.autonomousEndPose == null) {
                drivebase.setFieldStartPose(
                        blueDefaultEndPose.x(),
                        blueDefaultEndPose.y(),
                        Math.toDegrees(blueDefaultEndPose.heading())
                );
            }
        }


        myTe.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        myTe.addData("Pose Source", OpModeStorage.autonomousEndPose != null ? "FROM AUTO" : "DEFAULT END POSE");
        myTe.addData("Start X", "%.1f", drivebase.getFieldX());
        myTe.addData("Start Y", "%.1f", drivebase.getFieldY());
        myTe.addData("Start Heading (deg)", "%.1f", drivebase.getFieldHeadingDeg());
        myTe.addLine("DPAD LEFT = RED | DPAD RIGHT = BLUE");
        myTe.update();
    }


    @Override
    public void start() {
        manualHeadingOffsetRad = 0.0;
    }


    @Override
    public void loop() {
        drivebase.updatePinpoint();


        double currentHeadingRad = Math.toRadians(drivebase.getFieldHeadingDeg());
        double allianceOffsetRad = Math.toRadians(
                isRedAlliance ? redHeadingOffsetDeg : blueHeadingOffsetDeg
        );


        // Press Options on Gamepad 1 while pointing robot away from driver to zero field-centric
        if (gamepad1.optionsWasPressed()) {
            manualHeadingOffsetRad = currentHeadingRad - allianceOffsetRad;
        }


        double fieldCentricHeading = currentHeadingRad - allianceOffsetRad - manualHeadingOffsetRad;


        // Matched to Drivebase.drive() & Drivebase.driveFieldCentric() signs
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        drivebase.driveFieldCentric(forward, strafe, turn, fieldCentricHeading);


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
        myTe.addData("Heading (deg)", "%.1f", drivebase.getFieldHeadingDeg());
        myTe.addData("X", "%.1f", drivebase.getFieldX());
        myTe.addData("Y", "%.1f", drivebase.getFieldY());
        myTe.addData("Shooter P Velocity", velocityP);
        myTe.addData("Shooter N Velocity", velocityN);
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
        double adjustedPosition = Math.max(0.0, Math.min(1.0, position));
        gateP.setPosition(adjustedPosition);
    }


    private void closeGates() {
        gateN.setPosition(gateNClosed);
        setGatePPosition(gatePClosed);
    }


    @Override
    public void stop() {
        drivebase.stopRobot();
        intake.setPower(0);
        intake.closeFlush();
        shooter.stop();
        closeGates();
    }
}