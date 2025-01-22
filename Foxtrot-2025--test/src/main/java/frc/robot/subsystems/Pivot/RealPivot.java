package frc.robot.subsystems.Pivot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ShooterConstants;

public class RealPivot implements PivotIO{
    private static final int PivotMotorId = 62;
    private final SparkMax pivotMotor;
    private final CANcoder absoluteEncoder = new CANcoder(ShooterConstants.kAbsoluteEncoderId);
    private final PIDController pivotPidController = new PIDController(ShooterConstants.kPivotP, ShooterConstants.kPivotI, ShooterConstants.kPivotD);



public RealPivot(int PivotMotorID){
    pivotMotor = new SparkMax(PivotMotorId, MotorType.kBrushless);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
pivotConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
pivotConfig.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
pivotConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);



pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    CANcoderConfiguration config = new CANcoderConfiguration();
                config.MagnetSensor.MagnetOffset = ShooterConstants.kAbsoluteEncoderOffset;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(config);

        pivotPidController.setTolerance(ShooterConstants.kPivotToleranceRad);
        pivotPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        
        }

        @Override
    public void setDesiredAngle(double angle){
        angle = Math.toRadians(angle);
        if (Math.abs(angle) < ShooterConstants.kPivotToleranceRad) {
            stop();
            return;
        }

        angle = MathUtil.clamp(angle, ShooterConstants.kMinShooterAngleRad, ShooterConstants.kMaxShooterAngleRad);

        pivotMotor.set(pivotPidController.calculate(getAngle(), angle));
    }

    @Override
    public double getAngle(){
        return absoluteEncoder.getPosition().getValueAsDouble() * ShooterConstants.kPivotMotorRot2Rad;
    }

    public double getAbsolutePosition(){
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    public void resetEncoders(){
        absoluteEncoder.setPosition(getAbsolutePosition() / 360);
    }

    
    @Override
    public void stop(){
        pivotMotor.set(0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(absoluteEncoder.getPosition(), absoluteEncoder.getAbsolutePosition()).isOK();

        inputs.positionRads = getAngle();
        inputs.absoluteEncoderPositionRads = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());
        inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(absoluteEncoder.getVelocity());
        inputs.appliedVolts = pivotMotor.getBusVoltage();
        inputs.supplyCurrentAmps = pivotMotor.getOutputCurrent();
        inputs.tempCelcius = pivotMotor.getMotorTemperature();
    }


    



    






}
    

