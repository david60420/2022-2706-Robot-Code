package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.config.Config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

public class DriveBase2020 extends DriveBase {
    WPI_TalonSRX leftMaster, rightMaster, climberTalon;
    BaseMotorController leftSlave, rightSlave;

    // DriveBase2020 is a singleton class as it represents a physical subsystem
    private static DriveBase2020 currentInstance;

    public double motorCurrent; //variable to display motor current levels
    public boolean motorLimitActive = false; //states if motor current is actively being limited
    
    public DriveBase2020() {
        leftMaster = new WPI_TalonSRX(Config.LEFT_FRONT_MOTOR);
        rightMaster = new WPI_TalonSRX(Config.RIGHT_FRONT_MOTOR);

        // Check whether to construct a victor or a talon
        if (Config.LEFT_SLAVE_ISVICTOR) {
            leftSlave = new WPI_VictorSPX(Config.LEFT_REAR_MOTOR);
        } else {
            leftSlave = new WPI_TalonSRX(Config.LEFT_REAR_MOTOR);
        }
        if (Config.RIGHT_SLAVE_ISVICTOR) {
            rightSlave = new WPI_VictorSPX(Config.RIGHT_REAR_MOTOR);
        } else {
            rightSlave = new WPI_TalonSRX(Config.RIGHT_REAR_MOTOR);
        }


        climberTalon = new WPI_TalonSRX(Config.CLIMBER_TALON);
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        resetMotors();
        setTalonConfigurations();
  
        setCoastMode();

        if (Config.PIGEON_ID != -1) {
            pigeon = new PigeonIMU(new WPI_TalonSRX(Config.PIGEON_ID));
            pigeon.setFusedHeading(0d, Config.CAN_TIMEOUT_LONG);
        }


    }

    public double getMotorCurrent() {
        //Get motor supply current, send it to shuffleboard, and return it.
        motorCurrent = (leftMaster.getSupplyCurrent() + rightMaster.getSupplyCurrent())/2;
        SmartDashboard.putNumber("Avg Motor Current", motorCurrent);
        return(motorCurrent); //Returns average motor current draw.
    }

    public boolean isMotorLimitActive() {
        //Checks if motor currents are at or above the continuous limit (checks if current limiting is imminent or ongoing)
        //This method does not limit motor current. It monitors current for driver feedback purposes.
        if (((leftMaster.getSupplyCurrent() >= Config.CONTIN_CURRENT_AMPS) == true) || ((rightMaster.getSupplyCurrent() >= Config.CONTIN_CURRENT_AMPS) == true)) {
            motorLimitActive = true;
        }
        else {
            motorLimitActive = false;
        }

        //Tell shuffleboard if current limting is active and return the result.
        SmartDashboard.putBoolean("MotorCurrentLimit T/F", motorLimitActive);
        return(motorLimitActive);
    }

    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new DriveBase2020();
        }
    }

    public static DriveBase2020 getInstance() {
        init();
        return currentInstance;
    }
    
    @Override
    public void stopMotors() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();

        leftSlave.neutralOutput();
        rightSlave.neutralOutput();
    }
    
    @Override
    protected void resetMotors() {
        leftMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightMaster.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        leftSlave.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightSlave.configFactoryDefault(Config.CAN_TIMEOUT_LONG);

        leftMaster.configPeakCurrentLimit(0);
        leftMaster.configPeakCurrentDuration(0);
        leftMaster.configContinuousCurrentLimit(0);
        rightMaster.configPeakCurrentLimit(0);
        rightMaster.configPeakCurrentDuration(0);
        rightMaster.configContinuousCurrentLimit(0);


        this.followMotors();
    }

    private void setTalonConfigurations() {
        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration(); 

        // Put most settings for talon here
        talonConfig.neutralDeadband = Config.DRIVE_OPEN_LOOP_DEADBAND;
        

        //Current limiting for drivetrain master motors.
        if (Config.MOTOR_CURRENT_LIMIT == true) {
            talonConfig.peakCurrentLimit = Config.PEAK_CURRENT_AMPS;
            talonConfig.peakCurrentDuration = Config.PEAK_TIME_MS;
            talonConfig.continuousCurrentLimit = Config.CONTIN_CURRENT_AMPS;
        } else { 
            //If MOTOR_CURRENT_LIMIT is not true, remove talon current limits, just to be safe.
            talonConfig.peakCurrentLimit = 0;
            talonConfig.peakCurrentDuration = 0;
            talonConfig.continuousCurrentLimit = 0;
        }
        
        // Config all talon settings - returns worst error
        ErrorCode leftMasterError = leftMaster.configAllSettings(talonConfig);
        ErrorCode rightMasterError = rightMaster.configAllSettings(talonConfig);

        if (leftMasterError != null) 
            logErrorCode(leftMasterError, "LeftMaster");
        if (rightMasterError != null)
            logErrorCode(rightMasterError, "RightMaster");

        // Config the encoder and check if it worked
        ErrorCode e1 = leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ErrorCode e2 = rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ErrorCode e3 = leftSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ErrorCode e4 = rightSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        if (e1 != ErrorCode.OK || e2 != ErrorCode.OK || e3 != ErrorCode.OK || e4 != ErrorCode.OK)
            this.state = DriveBaseState.Degraded;

        // Set the motor inversions
        leftMaster.setInverted(Config.LEFT_FRONT_INVERTED);
        rightMaster.setInverted(Config.RIGHT_FRONT_INVERTED);
        leftSlave.setInverted(Config.LEFT_REAR_INVERTED);
        rightSlave.setInverted(Config.RIGHT_REAR_INVERTED);

        // set the encoder inversions
        leftMaster.setSensorPhase(Config.DRIVETRAIN_LEFT_SENSORPHASE);
        rightMaster.setSensorPhase(Config.DRIVETRAIN_RIGHT_SENSORPHASE);
    }

    private void logErrorCode(ErrorCode e, String motorName) {
        String[] errorCategories = new String[]{"CAN-Related", "UserSpecifiedGeneral", "Signal", "Gadgeteer Port Error Codes", 
                    "Gadgeteer Module Error Codes", "API", "Higher Level", "CAN Related", "General", "Simulation"};
        String errorCategory;
        if (e.value >= -8 && e.value <= 10) 
            errorCategory = errorCategories[0];
        else if(e.value == -100) 
            errorCategory = errorCategories[1];
        else if(e.value == -200 || e.value == -201) 
            errorCategory = errorCategories[2];
        else if(e.value == -300 || e.value == -301)
            errorCategory = errorCategories[3];
        else if(e.value == -400 || e.value == -401 || e.value == -402)
            errorCategory = errorCategories[4];
        else if(e.value >= -505 && e.value <= -500)
            errorCategory = errorCategories[5];
        else if(e.value == -600 || e.value == -601)
            errorCategory = errorCategories[6];
        // skip errorCategories[7] b/c its included in errorCategories[0]
        else if(e.value >= 100 && e.value <= 110)
            errorCategory = errorCategories[8];
        else if(e.value == 200 || e.value == 201 || e.value == 202)
            errorCategory = errorCategories[9];
        else
            errorCategory = "Unknown Category";

        String logString = String.format("%s - %s - %s", motorName, errorCategory, e.name());
        // Log this string. For now println
        System.out.println(logString);
        
    }

    public void setCoastMode() {
        leftMaster.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave.setNeutralMode(NeutralMode.Coast);
        rightSlave.setNeutralMode(NeutralMode.Coast);
    }
    
    @Override
    protected void followMotors() {
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }
    
    @Override
    protected void driveModeUpdated(DriveMode mode) {
        this.stopMotors();
        if (mode == DriveMode.OpenLoopVoltage) {
            ErrorCode e1 = leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            ErrorCode e2 = rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            ErrorCode e3 = leftSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            ErrorCode e4 = rightSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            
            if (e1 != ErrorCode.OK || e2 != ErrorCode.OK || e3 != ErrorCode.OK || e4 != ErrorCode.OK)
                this.state = DriveBaseState.Degraded;
                
            leftMaster.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
            rightMaster.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
            leftSlave.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
            rightSlave.configNeutralDeadband(Config.DRIVE_OPEN_LOOP_DEADBAND);
        } else if (mode == DriveMode.Disabled) {
            this.resetMotors();
        }
    }
    
}
