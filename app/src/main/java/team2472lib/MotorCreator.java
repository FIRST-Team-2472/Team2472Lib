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


/**
 * Class designed to setup both Talon and SparkMax motors with their configs
 */
public class MotorCreator {
    int ID, statorLimit, supplyLimit;
    boolean inverted;
    MotorType type;
    IdleMode idleMode;

    /**
     * Creates a new object containing the config settings for the motor
     * @param ID The device ID.
     * @param statorLimit The current limit (in Amps) flowing inside the motor coils. 
     * Protects the physical motor from overheating and gearboxes from excessive torque spikes.
     * @param supplyLimit The maximum current (in Amps) the controller is allowed to draw from the battery. 
     * Protects the main electrical system from brownouts.
     * @param type The motor type connected to the controller. Brushless motor wires must be connected
     * to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     * connected to the Red and Black terminals only.
     * @param inverted True for Clockwise positive.
     * @param idleMode Coast or brake.
     */

     /**
     * Creates a new configuration profile containing the settings for a motor.
     * @param ID The CAN or PWM device ID.
     * @param statorLimit The current limit (in Amps) flowing inside the motor coils. 
     * Protects the physical motor from overheating and gearboxes from torque spikes.
     * @param supplyLimit The maximum current (in Amps) the controller is allowed to draw from the battery. 
     * Protects the main electrical system from brownouts.
     * @param type The motor type connected to the controller. Brushless motor wires must be connected
     * to their matching colors and the hall sensor plugged in. Brushed motors must be
     * connected to the Red and Black terminals only.
     * @param inverted True to invert the motor orientation (Clockwise positive), false for default (Counter-Clockwise positive).
     * @param useBrakeMode Coast or brake.
     */
    public MotorCreator(int ID, int statorLimit, int supplyLimit, MotorType type, boolean inverted, IdleMode idleMode){
        this.ID = ID;
        this.statorLimit = statorLimit;
        this.supplyLimit = supplyLimit;
        this.inverted = inverted;
        this.type = type;
        this.idleMode = idleMode;
    }

    /**
     * Creates and configures a SparkMax Motor.
     * @return A fully configured SparkMax motor instance.
     */
    public SparkMax getSparkMax() {
        SparkMax motor = new SparkMax(ID, type);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted).smartCurrentLimit(statorLimit).secondaryCurrentLimit(supplyLimit)
            .idleMode(idleMode);

        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        return motor;
    }

    /**
     * Creates and configures a SparkMax Motor.
     * @param resetMode Whether to reset safe parameters before setting the configuration
     * @param persistMode Whether to persist the parameters after setting the configuration.
     * @return A fully configured SparkMax motor instance.
     */
    public SparkMax getSparkMax(SparkMax.ResetMode resetMode, SparkMax.PersistMode persistMode) {
        SparkMax motor = new SparkMax(ID, type);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted).smartCurrentLimit(statorLimit).secondaryCurrentLimit(supplyLimit)
            .idleMode(idleMode);

        motor.configure(config, resetMode, persistMode);
        return motor;
    }

    /**
     * Creates and configures a TalonFX Motor.
     * @return A fully configured TalonFX motor instance.
     */
    public TalonFX geTalonFX() {
        TalonFX motor = new TalonFX(ID);

        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
            outputConfig.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            outputConfig.NeutralMode = NeutralModeValue.valueOf(idleMode.value);
        
        CurrentLimitsConfigs limitConfig = new CurrentLimitsConfigs();
            limitConfig.StatorCurrentLimit = statorLimit;
            limitConfig.SupplyCurrentLimit = supplyLimit;

        motor.getConfigurator().apply(outputConfig);
        motor.getConfigurator().apply(limitConfig);

        return motor;
    }


}
