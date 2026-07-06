package team2472lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorCreator {
    int ID, statorCurrentLimit, supplyCurrentLimit;
    boolean setInverted;
    MotorType motorType;
    IdleMode idleMode;
    public MotorCreator(int ID, int statorCurrentLimit, int supplyCurrentLimit, MotorType motorType, boolean setInverted, IdleMode idleMode){
        this.ID = ID;
        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;
        this.setInverted = setInverted;
        this.motorType = motorType;
        this.idleMode = idleMode;
    }

    public SparkMax getSparkMax() {
        SparkMax motor = new SparkMax(ID, motorType);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(setInverted).smartCurrentLimit(statorCurrentLimit).secondaryCurrentLimit(supplyCurrentLimit)
            .idleMode(idleMode);

        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        return motor;
    }

    public SparkMax getSparkMax(SparkMax.ResetMode resetMode, SparkMax.PersistMode persistMode) {
        SparkMax motor = new SparkMax(ID, motorType);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(setInverted).smartCurrentLimit(statorCurrentLimit).secondaryCurrentLimit(supplyCurrentLimit)
            .idleMode(idleMode);

        motor.configure(config, resetMode, persistMode);
        return motor;
    }

    public TalonFX geTalonFX() {
        TalonFX motor = new TalonFX(ID);

        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
            outputConfig.Inverted = setInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            outputConfig.NeutralMode = NeutralModeValue.valueOf(idleMode.value);
        
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
            limitConfig.StatorCurrentLimit = statorCurrentLimit;
            limitConfig.SupplyCurrentLimit = supplyCurrentLimit;

        motor.getConfigurator().apply(outputConfig);
        motor.getConfigurator().apply(limitConfig);
    
        return motor;
    }


}
