package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class Drivebase {


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private GoBildaPinpointDriver pinpoint;


    private double startFieldX = 0.0;
    private double startFieldY = 0.0;
    private double startFieldHeadingDeg = 0.0;


    public Drivebase(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }


    public Drivebase(HardwareMap hardwareMap, boolean resetPinpoint) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        pinpoint.setOffsets(7.286581204632136, 0.25816200286384644, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );


        if (resetPinpoint) {
            pinpoint.resetPosAndIMU();
        }
    }


    public void updatePinpoint() {
        pinpoint.update();
    }


    /**
     * Call in init() ONLY (resets IMU while robot is completely still).
     */
    public void setFieldStartPose(double fieldX, double fieldY, double fieldHeadingDeg) {
        pinpoint.resetPosAndIMU();
        this.startFieldX = fieldX;
        this.startFieldY = fieldY;
        this.startFieldHeadingDeg = fieldHeadingDeg;
    }


    /** Global Field X (inches) matching Pedro Pathing coordinates */
    public double getFieldX() {
        Pose2D pos = pinpoint.getPosition();
        double localX = pos.getX(DistanceUnit.INCH);
        double localY = pos.getY(DistanceUnit.INCH);
        double rad = Math.toRadians(startFieldHeadingDeg);
        return startFieldX + (localX * Math.cos(rad) - localY * Math.sin(rad));
    }


    /** Global Field Y (inches) matching Pedro Pathing coordinates */
    public double getFieldY() {
        Pose2D pos = pinpoint.getPosition();
        double localX = pos.getX(DistanceUnit.INCH);
        double localY = pos.getY(DistanceUnit.INCH);
        double rad = Math.toRadians(startFieldHeadingDeg);
        return startFieldY + (localX * Math.sin(rad) + localY * Math.cos(rad));
    }


    /** Global Field Heading (degrees, CCW positive) matching Pedro Pathing coordinates */
    public double getFieldHeadingDeg() {
        return normalizeAngleDeg(startFieldHeadingDeg + pinpoint.getHeading(AngleUnit.DEGREES));
    }


    /** Raw Pinpoint IMU heading in radians (preserved from Auto into TeleOp) */
    public double getRawHeadingRadians() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }


    // --- TeleOp Methods ---


    public void drive(double drive, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(drive) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );


        double frontLeftPower = (drive + strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


    public void driveFieldCentric(double drive, double strafe, double turn, double headingRadians) {
        double rotX = strafe * Math.cos(-headingRadians) - drive * Math.sin(-headingRadians);
        double rotY = strafe * Math.sin(-headingRadians) + drive * Math.cos(-headingRadians);
        drive(rotY, rotX, turn);
    }


    public void turnRobot(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }


    public void stopRobot() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    public void resetAim() {
        stopRobot();
    }


    // --- Field-Coordinate Movement Methods ---


    /**
     * Turns in place to an exact global field heading (degrees, CCW positive).
     */
    public boolean turnToFieldHeading(double targetHeadingDeg, double maxPower) {
        double error = normalizeAngleDeg(targetHeadingDeg - getFieldHeadingDeg());


        if (Math.abs(error) <= 2.5) {
            stopRobot();
            return true;
        }


        double rawPower = -error * 0.014;
        double clamped = Math.max(-maxPower, Math.min(maxPower, rawPower));
        if (Math.abs(clamped) < 0.16) {
            clamped = Math.copySign(0.16, clamped);
        }


        drive(0, 0, clamped);
        return false;
    }


    /**
     * Drives directly to exact global field coordinates (targetX, targetY, targetHeadingDeg).
     */
    public boolean driveToFieldPose(double targetX, double targetY, double targetHeadingDeg, double maxPower) {
        double currentX = getFieldX();
        double currentY = getFieldY();
        double currentHeadingDeg = getFieldHeadingDeg();


        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double distError = Math.hypot(errorX, errorY);
        double headingError = normalizeAngleDeg(targetHeadingDeg - currentHeadingDeg);


        boolean atPos = distError <= 1.75;
        boolean atHeading = Math.abs(headingError) <= 2.5;


        if (atPos && atHeading) {
            stopRobot();
            return true;
        }


        double robotForward = 0.0;
        double robotStrafeRight = 0.0;


        // Only apply translation power if we haven't reached the position tolerance yet
        if (!atPos) {
            double speed = Math.min(maxPower, Math.max(0.22, distError * 0.05));
            double fieldVx = (errorX / distError) * speed;
            double fieldVy = (errorY / distError) * speed;


            double rad = Math.toRadians(currentHeadingDeg);
            robotForward = fieldVx * Math.cos(rad) + fieldVy * Math.sin(rad);
            double robotLeft = -fieldVx * Math.sin(rad) + fieldVy * Math.cos(rad);
            robotStrafeRight = -robotLeft;
        }


        // Turn power (-turn = CCW in Drivebase.drive)
        double turnPower = 0.0;
        if (!atHeading) {
            turnPower = Math.max(-0.35, Math.min(0.35, -headingError * 0.014));
            if (Math.abs(turnPower) < 0.14 && distError < 4.0) {
                turnPower = Math.copySign(0.14, turnPower);
            }
        }


        drive(-robotForward, -robotStrafeRight, turnPower);
        return false;
    }


    private double normalizeAngleDeg(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}