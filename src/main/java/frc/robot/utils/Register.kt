package frc.robot.utils

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.TEST_MODE

/**
 * Type alias for a pair consisting of a command name and a command.
 */
typealias NamedCommand = Pair<String, Command>

/**
 * Type alias for a pair consisting of a button and a command.
 */
typealias Binding = Pair<Button, Command>

/**
 * Type alias for a map of buttons to joystick buttons.
 */
typealias ButtonMap = Map<Button, JoystickButton>

typealias Log = Pair<String, Any>

/**
 * Object that provides a utility function to register things.
 */
object Register {
    /**
     * Creates a [Binding] from a button and a command.
     *
     * @param button The button to bind.
     * @param command The command to execute when the button is pressed.
     * @return A [Binding] consisting of the button and the command.
     */
    @JvmStatic
    fun bind(
        button: Button,
        command: Command,
    ) = Binding(button, command)

    /**
     * Binds multiple buttons to their respective commands.
     *
     * @receiver A map of buttons to joystick buttons.
     * @param bindings Vararg parameter of bindings where each binding consists of a button and a command.
     */
    @JvmStatic
    @SafeVarargs
    fun ButtonMap.bindings(vararg bindings: Binding) =
        bindings.forEach {
            this[it.first]?.onTrue(it.second) ?: run {
                println("Missing binding for button ${it.first} in list $this")
            }
        }

    /**
     * Creates a [NamedCommand] from a command name and a command.
     *
     * @param string The name of the command.
     * @param command The command to be associated with the name.
     * @return A [NamedCommand] consisting of the command name and the command.
     */
    @JvmStatic
    fun cmd(
        string: String,
        command: Command,
    ) = NamedCommand(string, command)

    /**
     * Registers multiple commands with their respective names.
     *
     * @param commands Vararg parameter of pairs where the first element is the command name
     * and the second element is the command itself.
     */
    @JvmStatic
    @SafeVarargs
    fun commands(vararg commands: NamedCommand) =
        commands.forEach {
            NamedCommands.registerCommand(it.first, it.second)
        }

    /**
     * Utility class for logging. Provides methods to update PID
     * values, retrieve double values, create pairs, and perform test logging.
     */
    object Dash {
        private val capturedLogs = mutableListOf<Log>()

        /**
         * Logs a key-value pair.
         *
         * @param string The key associated with the value to log.
         * @param value The value to log.
         */
        @JvmStatic
        fun log(
            string: String,
            value: Any,
        ) {
            capturedLogs.add(Log(string, value))
        }

        /**
         * Logs multiple values within the provided block context.
         *
         * This function captures logs generated within the block and logs them all at once
         * after the block has been executed. It uses a custom `LogContext` to temporarily
         * override the log method to capture logs instead of immediately logging them.
         *
         * @param block The block of code within which logs will be captured.
         */
        @JvmStatic
        fun logs(block: Runnable) {
            capturedLogs.clear()

            block.run()

            logs(*capturedLogs.toTypedArray())
        }

        /**
         * Logs multiple values with their respective keys based on the type of each value.
         *
         * @param logs Vararg parameter of pairs where the first element is the key and the second element is the value to log.
         */
        @JvmStatic
        @SafeVarargs
        fun logs(vararg logs: Log) {
            logs.forEach { (key, value) ->
                when (value) {
                    is Double -> logs(key, value)
                    is Int -> logs(key, value)
                    is Boolean -> logs(key, value)
                    is String -> logs(key, value)
                    is WPISerializable -> logs(key, value)
                    else -> println("Unsupported log type for key $key")
                }
            }
        }

        /**
         * Logs a double value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The double value to log.
         */
        @JvmStatic
        fun logs(
            key: String?,
            value: Double,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, value)
            }
        }

        /**
         * Logs an integer value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The integer value to log.
         */
        @JvmStatic
        fun logs(
            key: String?,
            value: Int,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, value)
            }
        }

        /**
         * Logs a boolean value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The boolean value to log.
         */
        @JvmStatic
        fun logs(
            key: String?,
            value: Boolean,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, value)
            }
        }

        /**
         * Logs a String value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The String value to log.
         */
        @JvmStatic
        fun logs(
            key: String?,
            value: String?,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, value)
            }
        }

        /**
         * Logs a WPISerializable value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The WPISerializable value to log.
         */
        @JvmStatic
        fun <T : WPISerializable?> logs(
            key: String?,
            value: T,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, value)
            }
        }

        /**
         * Logs a SwerveModuleState value with a specified key if the system is in test mode.
         *
         * @param key The key associated with the value to log.
         * @param value The SwerveModuleState value to log.
         */
        @JvmStatic
        fun <T : StructSerializable?> logs(
            key: String?,
            vararg value: T,
        ) {
            if (TEST_MODE) {
                org.littletonrobotics.junction.Logger
                    .recordOutput(key, *value)
            }
        }
    }
}
