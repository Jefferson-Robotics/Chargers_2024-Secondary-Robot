package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX m_leftLeader = new TalonFX(LEFT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_leftFollower = new TalonFX(LEFT_FOLLOWER_ID, CANBUS_NAME);
    private final TalonFX m_rightLeader = new TalonFX(RIGHT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_rightFollower = new TalonFX(RIGHT_FOLLOWER_ID, CANBUS_NAME);

    private final TalonFXSimState m_leftSimState = m_leftLeader.getSimState();
    private final TalonFXSimState m_rightSimState = m_rightLeader.getSimState();
    private final TalonFXSimState m_leftFollowerSimState = m_leftFollower.getSimState();
    private final TalonFXSimState m_rightFollowerSimState = m_rightFollower.getSimState();

    private final DutyCycleOut m_leftOut = new DutyCycleOut(0); // Initialize with 0% output
    private final DutyCycleOut m_rightOut = new DutyCycleOut(0); // Initialize with 0% output

    /*
     * These numbers are an example AndyMark Drivetrain with some additional weight.
     * This is a fairly light robot.
     * Note you can utilize results from robot characterization instead of
     * theoretical numbers.
     * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
     * characterization/introduction.html#introduction-to-robot-characterization
     */
    private final double kGearRatio = 10.71;
    private final double kWheelRadiusInches = 3;

    public double forwardSpeed;
    public double rotationSpeed;

    /* Simulation model of the drivetrain */
    private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500Foc(2), // 2 CIMS on each side of the drivetrain.
        kGearRatio, // Standard AndyMark Gearing reduction.
        2.1, // MOI of 2.1 kg m^2 (from CAD model).
        26.5, // Mass of the robot is 26.5 kg.
        Units.inchesToMeters(kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
        0.546, // Distance between wheels is _ meters.

        /*
         * The standard deviations for measurement noise:
         * x and y: 0.001 m
         * heading: 0.001 rad
         * l and r velocity: 0.1 m/s
         * l and r position: 0.005 m
         */
        /* Uncomment the following line to add measurement noise. */
        null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );
    /*
     * Creating my odometry object. Here,
     * our starting pose is 5 meters along the long end of the field and in the
     * center of the field along the short end, facing forward.
     */

    public DriveSubsystem() {
        /* Initialize all the devices */
        initializeLeftDriveTalonFX(m_leftLeader.getConfigurator());
        initializeLeftDriveTalonFX(m_leftFollower.getConfigurator());
        initializeRightDriveTalonFX(m_rightLeader.getConfigurator());
        initializeRightDriveTalonFX(m_rightFollower.getConfigurator());

        /* Set followers to follow leader */
        m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
        m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

        /* Make sure all critical signals are synchronized */
        /*
         * Setting all these signals to 100hz means they get sent at the same time if
         * they're all on a CANivore
         */
        m_leftLeader.getPosition().setUpdateFrequency(100);
        m_rightLeader.getPosition().setUpdateFrequency(100);

        /*
         * Set the update frequency of the main requests to 0 so updates are sent
         * immediately in the arcadeDrive method
         */
        m_leftOut.UpdateFreqHz = 0;
        m_rightOut.UpdateFreqHz = 0;

        /* Currently in simulation, we do not support FOC, so disable it while simulating */
        if (Utils.isSimulation()) {
            m_leftOut.EnableFOC = false;
            m_rightOut.EnableFOC = false;
        }

        /*
         * Set the orientation of the simulated TalonFX devices relative to the robot chassis.
         * WPILib expects +V to be forward. Specify orientations to match that behavior.
         */
        /* left TalonFXs are CCW+ */
        m_leftSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        m_leftFollowerSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        /* right TalonFXs are CW+ */
        m_rightSimState.Orientation = ChassisReference.Clockwise_Positive;
        m_rightFollowerSimState.Orientation = ChassisReference.Clockwise_Positive;
    }

    /**
     * Drive the robot using an arcade drive format.
     * <p>
     * <b>This must be called periodically</b> or else the control frames will not
     * get sent
     * out, resulting in the TalonFXs disabling
     * 
     * @param fwd Forward/Reverse output
     * @param rot Left/Right output
     */
    public void arcadeDrive(double fwd, double rot) {
        m_leftOut.Output = fwd + rot;
        m_rightOut.Output = fwd - rot;
        forwardSpeed = fwd;
        rotationSpeed = rot;

        m_leftLeader.setControl(m_leftOut);
        m_rightLeader.setControl(m_rightOut);
    }

    @Override
    public void periodic() {
        /*
         * This will get the simulated sensor readings that we set
         * in the previous article while in simulation, but will use
         * real values on the robot itself.
         */
    }

    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated devices */
        m_leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_leftFollowerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_rightFollowerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        /*
         * CTRE simulation is low-level, so SimState inputs
         * and outputs are not affected by user-level inversion.
         * However, inputs and outputs *are* affected by the mechanical
         * orientation of the device relative to the robot chassis,
         * as specified by the `orientation` field.
         *
         * WPILib expects +V to be forward. We have already configured
         * our orientations to match this behavior.
         */
        m_driveSim.setInputs(m_leftSimState.getMotorVoltage(),
                m_rightSimState.getMotorVoltage());

        /*
         * Advance the model by 20 ms. Note that if you are running this
         * subsystem in a separate thread or have changed the nominal
         * timestep of TimedRobot, this value needs to match it.
         */
        m_driveSim.update(0.02);

        /* Update all of our sensors. */
        m_leftSimState.setRawRotorPosition(
            metersToRotations(m_driveSim.getLeftPositionMeters())
        );
        m_leftSimState.setRotorVelocity(
            // This is OK, since the time base is the same
            metersToRotations(m_driveSim.getLeftVelocityMetersPerSecond())
        );
        m_rightSimState.setRawRotorPosition(
            metersToRotations(m_driveSim.getRightPositionMeters())
        );
        m_rightSimState.setRotorVelocity(
            // This is OK, since the time base is the same
            metersToRotations(m_driveSim.getRightVelocityMetersPerSecond())
        );
        m_leftFollowerSimState.setRawRotorPosition(
            metersToRotations(m_driveSim.getLeftPositionMeters())
        );
        m_leftFollowerSimState.setRotorVelocity(
            // This is OK, since the time base is the same
            metersToRotations(m_driveSim.getLeftVelocityMetersPerSecond())
        );
        m_rightFollowerSimState.setRawRotorPosition(
            metersToRotations(m_driveSim.getRightPositionMeters())
        );
        m_rightFollowerSimState.setRotorVelocity(
            // This is OK, since the time base is the same
            metersToRotations(m_driveSim.getRightVelocityMetersPerSecond())
        );
    }

    public StatusSignal<Double> getLeftPos() {
        return m_leftLeader.getPosition();
    }

    public StatusSignal<Double> getRightPos() {
        return m_rightLeader.getPosition();
    }

    /**
     * Initialize a left drive TalonFX device from the configurator object
     * 
     * @param cfg Configurator of the TalonFX device
     */
    private void initializeLeftDriveTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */
        toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.apply(toApply);

        /* And initialize position to 0 */
        cfg.setPosition(0);
    }

    /**
     * Initialize a right drive TalonFX device from the configurator object
     * 
     * @param cfg Configurator of the TalonFX device
     */
    private void initializeRightDriveTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.apply(toApply);

        /* And initialize position to 0 */
        cfg.setPosition(0);
    }

    /**
     * Initialize Pigeon2 device from the configurator object
     * 
     * @param cfg Configurator of the Pigeon2 device
     */

    private double metersToRotations(double meters) {
        /* Get circumference of wheel */
        final double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the rotations per meter traveled */
        final double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
        /* Now apply wheel rotations to input meters */
        double wheelRotations = wheelRotationsPerMeter * meters;
        /* And multiply by gear ratio to get rotor rotations */
        return wheelRotations * this.kGearRatio;
    }
}
