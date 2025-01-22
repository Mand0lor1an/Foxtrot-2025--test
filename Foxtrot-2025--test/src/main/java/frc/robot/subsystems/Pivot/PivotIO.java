package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    public double getAngle();
    public void setDesiredAngle(double angle);
    public void stop();

    default void updateInputs(PivotIOInputs inputs) {}


        @AutoLog
    class PivotIOInputs {
        public boolean motorConnected = true;

        public double positionRads = 0.0;
        public double absoluteEncoderPositionRads = 0.0;
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
        public boolean absoluteEncoderConnected = true;
    }
}

