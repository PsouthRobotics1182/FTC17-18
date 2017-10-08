package FineLib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by drew on 10/6/17.
 */
public class FineMotor {
    HardwareMap hwMap;
    private DcMotor motor;
    private MotorConfigurationType config;
    private double wheelRadius;
    private double linearSpeed = 0;
    private double rotationalSpeed = 0;
    private int startPos = 0;

    //the gear ratio after the output shaft of the motor
    public FineMotor(HardwareMap hwMap, String name, int gearRatio, DcMotorSimple.Direction direction) {
        this.hwMap = hwMap;
        motor = hwMap.get(DcMotor.class, name);
        config = motor.getMotorType().clone();
        config.setGearing(config.getGearing()*gearRatio);
        motor.setMotorType(config);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(direction);
    }

    public void setLinearSpeed(double linearSpeed) {
        this.linearSpeed = linearSpeed;
        this.rotationalSpeed = lin2rot(linearSpeed);

    }
    public void setRotationalSpeed(double rotationalSpeed){
        this.rotationalSpeed = rotationalSpeed;
        this.linearSpeed = rot2lin(rotationalSpeed);
    }
    public double maxRotationalSpeed() {
        return motor.getMotorType().getAchieveableMaxRPMFraction();
    }
    public void power() {
        double power = this.rotationalSpeed/(maxRotationalSpeed());
        motor.setPower(power);
    }
    public void power(double power) {
        motor.setPower(power);
    }
    public void stop() {
        motor.setPower(0);
    }
    public int getCurrentPosition() {
        return motor.getCurrentPosition()-startPos;
    }
    public void resetPosition() {
        startPos = motor.getCurrentPosition();
    }
    public MotorConfigurationType getMotorType() {
        return config;
    }
    private double lin2rot(double linSpeed) {
        double circumfrence = Math.PI * wheelRadius * 2;
        double rotations = linSpeed/circumfrence;
        double rotSpeed = rotations * 60;
        return rotSpeed;
    }
    private double rot2lin(double rotSpeed) {
        double circumfrence = Math.PI * wheelRadius * 2;
        rotSpeed = rotSpeed / 60;
        double linSpeed = rotSpeed * circumfrence;
        return linSpeed;
    }

}
