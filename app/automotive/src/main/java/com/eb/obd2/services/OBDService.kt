package com.eb.obd2.services

import android.util.Log
import com.eb.obd2.models.ConnectionStatus
import com.eb.obd2.models.RuntimeRecord
import com.eb.obd2.models.VehicleData
import com.eb.obd2.repositories.RecordRepository
import com.github.pires.obd.commands.ObdCommand
import com.github.pires.obd.commands.SpeedCommand
import com.github.pires.obd.commands.engine.RPMCommand
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.IOException
import java.time.LocalDateTime
import org.json.JSONObject
import javax.inject.Inject
import javax.inject.Singleton

/**
 * OBD Service to handle the OBD data communications.
 */
@Singleton
class OBDService @Inject constructor(
    private val connection: OBDConnection,
    private val recordRepository: RecordRepository
) {

    companion object {
        private const val TAG = "OBDService"
        private const val RECONNECT_DELAY = 5000L // 5 seconds between reconnection attempts
        private const val UPDATE_INTERVAL = 100L // 100ms between data updates
        
        // OBD-II PIDs
        private const val PID_RPM = "01 0C"
        private const val PID_SPEED = "01 0D"
        private const val PID_ENGINE_TEMP = "01 05"
        private const val PID_THROTTLE = "01 11"
        private const val PID_FUEL_LEVEL = "01 2F"
        private const val PID_FUEL_RATE = "01 5E"
        private const val PID_GEAR = "01 A4"  // Custom PID for current gear
        private const val PID_GEAR_POSITION = "01 A5"  // Custom PID for gear position (P,R,N,D)
        private const val PID_BRAKE = "01 10"  // Custom PID for brake position
    }

    private val _connectionStatus = MutableStateFlow(ConnectionStatus.DISCONNECTED)
    val connectionStatus: StateFlow<ConnectionStatus> = _connectionStatus.asStateFlow()

    private val _vehicleData = MutableStateFlow(VehicleData())
    val vehicleData: StateFlow<VehicleData> = _vehicleData.asStateFlow()

    private var serviceJob: Job? = null
    private val serviceScope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    private val commands = mutableSetOf<ObdCommand>()

    /** Commands for two-way communication */
    private object CommandTypes {
        const val SETTINGS = "CMD_SETTINGS"
        const val THROTTLE = "CMD_THROTTLE"
        const val BRAKE = "CMD_BRAKE"
        const val PROFILE = "CMD_PROFILE"
        const val SIMULATION = "CMD_SIM"
        const val SPEED = "CMD_SPEED"
    }

    /**
     * Emulator settings that can be adjusted
     */
    enum class EmulatorSetting(val id: String) {
        ENGINE_TEMP_FACTOR("engine_temp_factor"),
        FUEL_CONSUMPTION_FACTOR("fuel_consumption_factor"),
        RPM_RESPONSE_FACTOR("rpm_response_factor"),
        SIMULATION_SPEED("simulation_speed"),
        FAULT_SIMULATION("fault_simulation"),
        BATTERY_VOLTAGE_FACTOR("battery_voltage_factor")
    }

    /**
     * Command status for OBD service operations
     */
    private val _commandStatus = MutableStateFlow<CommandStatus>(CommandStatus.Idle)
    val commandStatus: StateFlow<CommandStatus> = _commandStatus.asStateFlow()

    init {
        registerCommand(RPMCommand(), SpeedCommand())
    }

    fun registerCommand(vararg command: ObdCommand): OBDService {
        commands.addAll(command)
        return this
    }

    fun startConnection() {
        serviceJob?.cancel()
        serviceJob = serviceScope.launch {
            _connectionStatus.value = ConnectionStatus.CONNECTING
            
            while (true) { // Keep trying to connect indefinitely
                try {
                    Log.i(TAG, "Attempting to connect to OBD")
                    connection.openConnection()
                    _connectionStatus.value = ConnectionStatus.CONNECTED
                    Log.i(TAG, "OBD connected successfully")

                    // Initialize the connection with ELM327 commands
                    initializeConnection()

                    // Start reading real-time data
                    try {
                        while (connection.isConnected()) {
                            val data = readVehicleData()
                            _vehicleData.value = data
                            delay(UPDATE_INTERVAL)
                        }
                    } catch (e: Exception) {
                        Log.e(TAG, "Error reading data", e)
                    }

                    // If we get here, connection was lost
                    _connectionStatus.value = ConnectionStatus.DISCONNECTED
                    connection.closeConnection()
                    
                } catch (e: Exception) {
                    Log.e(TAG, "Connection failed", e)
                    _connectionStatus.value = ConnectionStatus.DISCONNECTED
                }

                delay(RECONNECT_DELAY) // Wait before trying to reconnect
            }
        }
    }

    private suspend fun initializeConnection() {
        var retries = 0
        val maxRetries = 3
        
        while (retries < maxRetries) {
            try {
                // Send initialization commands
                val initCommands = listOf(
                    "ATZ",     // Reset
                    "ATE0",    // Echo off
                    "ATL0",    // Linefeeds off
                    "ATS0",    // Spaces off
                    "ATH0",    // Headers off
                    "ATSP0"    // Auto protocol
                )
                
                for (cmd in initCommands) {
                    connection.sendCommand(cmd)
                    val response = connection.readResponse()
                    if (response?.contains("OK") != true && response?.contains("ELM327") != true) {
                        throw IOException("Failed to initialize ELM327: $cmd -> $response")
                    }
                    delay(100) // Small delay between commands
                }
                return // Success
            } catch (e: Exception) {
                Log.e(TAG, "Error initializing connection (attempt ${retries + 1}/$maxRetries)", e)
                retries++
                if (retries < maxRetries) {
                    delay(1000) // Wait before retry
                } else {
                    throw e // Rethrow if all retries failed
                }
            }
        }
    }

    private suspend fun readVehicleData(): VehicleData {
        return try {
            // Read all PIDs with retries
            val rpm = readPIDValueWithRetry(PID_RPM)?.let { calculateRPM(it) } ?: 0f
            val speed = readPIDValueWithRetry(PID_SPEED)?.toFloat() ?: 0f
            val engineTemp = readPIDValueWithRetry(PID_ENGINE_TEMP)?.let { (it - 40).toFloat() } ?: 70f
            val throttle = readPIDValueWithRetry(PID_THROTTLE)?.let { (it * 100 / 255).toFloat() } ?: 0f
            val fuelLevel = readPIDValueWithRetry(PID_FUEL_LEVEL)?.let { (it * 100 / 255).toFloat() } ?: 0f
            val fuelRate = readPIDValueWithRetry(PID_FUEL_RATE)?.let { calculateFuelRate(it) } ?: 0f
            val gear = readPIDValueWithRetry(PID_GEAR)?.toInt() ?: 1
            val gearPosition = readPIDValueWithRetry(PID_GEAR_POSITION)?.let { decodeGearPosition(it) } ?: "N"
            val brake = readPIDValueWithRetry(PID_BRAKE)?.let { (it * 100 / 255).toFloat() } ?: 0f

            VehicleData(
                rpm = rpm,
                speed = speed,
                engineTemp = engineTemp,
                throttlePercentage = throttle,
                fuelLevel = fuelLevel,
                fuelConsumptionRate = fuelRate,
                gear = gear,
                gearPosition = gearPosition,
                brakePercentage = brake
            )
        } catch (e: Exception) {
            Log.e(TAG, "Error reading vehicle data", e)
            VehicleData() // Return default data on error
        }
    }

    private suspend fun readPIDValueWithRetry(pid: String, maxRetries: Int = 3): Int? {
        var retries = 0
        while (retries < maxRetries) {
            try {
                connection.sendCommand(pid)
                val response = connection.readResponse()
                val value = parseOBDResponse(response, pid.split(" ")[1])?.toIntOrNull(16)
                if (value != null) {
                    return value
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error reading PID $pid (attempt ${retries + 1}/$maxRetries)", e)
            }
            retries++
            if (retries < maxRetries) {
                delay(100) // Wait before retry
            }
        }
        return null
    }

    private fun parseOBDResponse(response: String?, pid: String): String? {
        if (response == null || response.trim().isEmpty()) return null
        
        try {
            // Remove any whitespace and split
            val parts = response.trim().split(" ")
            
            // Check for minimum valid response
            if (parts.size < 3 || parts[0] != "41" || parts[1] != pid) {
                Log.d(TAG, "Invalid response format for PID $pid: $response")
                return null
            }

            // Handle multi-byte responses
            return when (pid) {
                "0C", "5E" -> { // RPM and Fuel Rate are 2-byte values
                    if (parts.size < 4) {
                        Log.d(TAG, "Incomplete multi-byte response for PID $pid: $response")
                        return null
                    }
                    val a = parts[2].toIntOrNull(16) ?: 0
                    val b = parts[3].toIntOrNull(16) ?: 0
                    ((a * 256) + b).toString()
                }
                else -> parts[2] // Single byte response
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error parsing OBD response: $response for PID: $pid", e)
            return null
        }
    }

    private fun calculateRPM(value: Int): Float {
        return (value * 0.25).toFloat()
    }

    private fun calculateFuelRate(value: Int): Float {
        return (value * 0.05).toFloat()
    }

    private fun decodeGearPosition(value: Int): String {
        return when (value.toChar()) {
            'P' -> "P"
            'R' -> "R"
            'N' -> "N"
            'D' -> "D"
            else -> "N"
        }
    }

    fun stopConnection() {
        serviceJob?.cancel()
        serviceScope.launch {
            try {
                connection.closeConnection()
            } catch (e: Exception) {
                Log.e(TAG, "Error closing connection", e)
            }
            _connectionStatus.value = ConnectionStatus.DISCONNECTED
        }
    }

    /**
     * Send a settings adjustment command to the emulator
     */
    fun adjustSetting(setting: EmulatorSetting, value: Float): Boolean {
        return sendCommand(CommandTypes.SETTINGS, mapOf(
            "setting" to setting.id,
            "value" to value.toString()
        ))
    }

    /**
     * Send a throttle control command to the emulator
     */
    fun setThrottlePosition(position: Float): Boolean {
        val safePosition = position.coerceIn(0f, 1f)
        return sendCommand(CommandTypes.THROTTLE, mapOf("position" to safePosition.toString()))
    }

    /**
     * Set a simulated speed for the emulator
     */
    fun setSimulatedSpeed(speed: Float): Boolean {
        val safeSpeed = speed.coerceAtLeast(0f)
        return sendCommand(CommandTypes.SPEED, mapOf("value" to safeSpeed.toString()))
    }

    /**
     * Send a brake control command to the emulator
     */
    fun setBrakePosition(position: Float): Boolean {
        val safePosition = position.coerceIn(0f, 1f)
        return sendCommand(CommandTypes.BRAKE, mapOf("position" to safePosition.toString()))
    }

    /**
     * Trigger a simulation profile to run on the emulator
     */
    fun triggerSimulationProfile(profileId: String, parameters: Map<String, String> = emptyMap()): Boolean {
        val params = mutableMapOf("profile_id" to profileId)
        params.putAll(parameters)
        return sendCommand(CommandTypes.PROFILE, params)
    }

    /**
     * Start or stop a simulation sequence
     */
    fun controlSimulation(action: String, scenarioId: String? = null): Boolean {
        val params = mutableMapOf("action" to action)
        scenarioId?.let { params["scenario_id"] = it }
        return sendCommand(CommandTypes.SIMULATION, params)
    }

    /**
     * Send a general command to the emulator
     */
    private fun sendCommand(commandType: String, parameters: Map<String, String>): Boolean {
        if (connectionStatus.value != ConnectionStatus.CONNECTED) {
            _commandStatus.value = CommandStatus.Error("Not connected to emulator")
            return false
        }

        _commandStatus.value = CommandStatus.Sending

        try {
            // Format command as JSON
            val jsonCommand = buildCommandJson(commandType, parameters)
            
            // Get output stream from connection
            val outputStream = connection.outputStream
            
            // Send command
            outputStream.write(jsonCommand.toByteArray())
            outputStream.flush()
            
            _commandStatus.value = CommandStatus.Success()
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to send command: $e")
            _commandStatus.value = CommandStatus.Error("Command failed: ${e.message}")
            return false
        }
    }

    /**
     * Build a JSON command string
     */
    private fun buildCommandJson(commandType: String, parameters: Map<String, String>): String {
        val jsonObject = JSONObject()
        try {
            jsonObject.put("type", commandType)
            
            // Add parameters as a nested object
            val paramsObject = JSONObject()
            parameters.forEach { (key, value) ->
                paramsObject.put(key, value)
            }
            jsonObject.put("params", paramsObject)
            
            // Add timestamp
            jsonObject.put("timestamp", System.currentTimeMillis())
        } catch (e: Exception) {
            Log.e(TAG, "Error building JSON command: $e")
        }
        return jsonObject.toString()
    }
}

/**
 * Status of a command execution
 */
sealed class CommandStatus {
    object Idle : CommandStatus()
    object Sending : CommandStatus()
    data class Success(val message: String = "Command completed successfully") : CommandStatus()
    data class Error(val message: String) : CommandStatus()
}