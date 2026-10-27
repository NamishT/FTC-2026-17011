package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
public class Drivebase {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private limeLightV2 lime;
    public Drivebase(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        lime = new limeLightV2(hardwareMap);
    }
    public void drive(double drive, double strafe, double turn){
        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1.0);
        double frontLeftPower = (drive+strafe+turn)/denominator;
        double frontRightPower = (drive-strafe-turn)/denominator;
        double backLeftPower = (drive-strafe+turn)/denominator;
        double backRightPower = (drive+strafe-turn)/denominator;
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
    }
    public boolean turnToAprilTag(int allianceType) {
        double[][] tags = lime.limeLight(allianceType);
        double closestDistance = Double.MAX_VALUE;
        double targetAngle = 0;
        boolean foundTag = false;
        for (double[] tag : tags) {
            if (tag == null || tag.length < 3) {
                continue;
            }
            if (tag[0] < closestDistance) {
                closestDistance = tag[0];
                targetAngle = tag[1];
                foundTag = true;
            }
        }

// Proportional turning control
        double turnPower = 0.02 * targetAngle;

// Minimum power to help overcome friction
        if (Math.abs(targetAngle) >= 1.5) {
            if (Math.abs(turnPower) < 0.12) {
                turnPower = Math.copySign(0.12, turnPower);
            }
        }

// Limit turning power
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
        if (Math.abs(targetAngle) < 1.5) {
            stopRobot();
            return false;
        } else {
            turnRobot(turnPower);
            return true;
        }
    }
    public void turnRobot(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(-power);
    }
    public void stopRobot() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}