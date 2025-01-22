package frc.robot.subsystems.Pivot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ShooterConstants;

public class RealPivot implements PivotIO{
    private static final int PivotMotorId = 62;
  private static final boolean PivotMotorReversed = true;
    private final SparkMax pivotMotor;




public RealPivot(int PivotMotorID){
    pivotMotor = new SparkMax(PivotMotorId, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
            .inverted(PivotMotorReversed)
          .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);}
  private final CANcoder absoluteEncoder = new CANcoder(ShooterConstants.kAbsoluteEncoderId);
        // Configure the motor
        pivotMotor.restoreFactoryDefaults();
        pivotMotor.setSmartCurrentLimit(40); // Set current limit
        pivotMotor.enableVoltageCompensation(12.0); // Enable voltage compensation
        pivotMotor.burnFlash(); // Save configurations to flash

        // Initialize and configure the absolute encoder
        absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setInverted(false); // Adjust based on encoder orientation
        absoluteEncoder.setZeroOffset(0.0); // Set zero offset
        absoluteEncoder.setPositionConversionFactor(GEAR_RATIO * 360.0); // Converts rotations to degrees
        absoluteEncoder.setVelocityConversionFactor(GEAR_RATIO * 360.0 / 60.0);

}
    

