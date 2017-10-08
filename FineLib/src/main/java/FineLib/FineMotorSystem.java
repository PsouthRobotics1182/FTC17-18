package FineLib;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class FineMotorSystem {

    private ArrayList<FineMotor> motors;
    private double wheelRadius;
    private double linearSpeed = 0;
    private double rotationalSpeed = 0;
    private double desiredRotations = 0;

    //the gear ratio after the output shaft of the motor
    public FineMotorSystem(HardwareMap hwMap, String nameBase, int gearRatio, int num, DcMotorSimple.Direction[] directions) {
        if (num != directions.length) {
            throw new IndexOutOfBoundsException("FineDcMotorSystem must be given equal number of directions and number of motors");
        }
        motors = new ArrayList<>();
        for (int i = 0; i < num; i ++) {
            FineMotor motor = new FineMotor(hwMap, nameBase + i, gearRatio, directions[i]);
            motors.add(motor);
        }
    }

    /**
     *
     * @param linearSpeed the
     */
    public void setLinearSpeed(double linearSpeed) {
        this.linearSpeed = linearSpeed;
        this.rotationalSpeed = lin2rot(linearSpeed);
        resetPositions();

    }
    public void setRotationalSpeed(double rotationalSpeed){
        this.rotationalSpeed = rotationalSpeed;
        this.linearSpeed = rot2lin(rotationalSpeed);
        resetPositions();
    }
    public double getMaxRotationalSpeed() {
        return motors.get(0).maxRotationalSpeed();
    }
    public void power() {
        double power = this.rotationalSpeed/(getMaxRotationalSpeed());
        power(power);
    }
    public void power(double power) {
        ArrayList<Integer> positions = new ArrayList<>();
        for (FineMotor motor: motors) {
            positions.add(Math.abs(motor.getCurrentPosition()));
        }
        double stdDev = stdev(positions);
        if (stdDev > 50)
            power = power * 0.9;

        for (FineMotor motor: motors) {
            motor.power(power);
        }
    }
    public boolean drive(double power) {
        if (getCurrentPosition() < rot2tick(desiredRotations)) {
            power(power);
            return true;
        } else {
            return false;
        }
    }
    public void setDistance(int mm) {
        double circumfrence = Math.PI * wheelRadius * 2;
        desiredRotations = mm / circumfrence;
        resetPositions();
    }
    public void setDesiredRotations(int desiredRotations) {
        this.desiredRotations = desiredRotations;
        resetPositions();
    }
    public void stop() {
        for (FineMotor motor: motors)
            motor.stop();
    }
    public int getCurrentPosition() {
        return Math.abs(motors.get(0).getCurrentPosition());
    }
    public void resetPositions() {
        for (FineMotor motor: motors)
            motor.resetPosition();
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
    private double rot2tick(double rots) {
        return rots * motors.get(0).getMotorType().getTicksPerRev();
    }
    private double stdev(ArrayList<Integer> values) {
        double powerSum1 = 0;
        double powerSum2 = 0;
        double stdev = 0;

        for (int i = 0; i <= values.size(); i++){
            powerSum1 += values.get(i);
            powerSum2 += Math.pow(values.get(i), 2);
            stdev = Math.sqrt(i*powerSum2 - Math.pow(powerSum1, 2))/i;
            //System.out.println(total[i]); // You specified that you needed to print
            // each value of the array
        }
        return stdev;
    }

}
