package com.eb.obd2.services

import com.eb.obd2.models.DrivingState
import com.eb.obd2.views.components.SystemStatus
import com.eb.obd2.views.components.VehicleWarning
import com.eb.obd2.views.components.WarningLevel
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.time.LocalDateTime
import java.util.UUID
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.random.Random

/**
 * Driving scenario types for simulation
 */
enum class DrivingScenarioType {
    CITY_DRIVING,        // Start-stop traffic with moderate speeds
    HIGHWAY_DRIVING,     // High speeds, steady RPM
    MOUNTAIN_DRIVING,    // Varying speeds, high RPM, engine temperature fluctuations
    AGGRESSIVE_DRIVING,  // Rapid acceleration/deceleration, high RPM 
    ENGINE_OVERHEAT,     // Gradually increasing engine temperature
    LOW_FUEL,            // Gradual fuel depletion
    LOW_BATTERY,         // Battery voltage drop scenario
    NORMAL_DRIVING       // Balanced, typical driving pattern
}

/**
 * Parameters for a driving scenario
 */
data class DrivingScenarioParams(
    val duration: Long = 120000, // Duration in milliseconds
    val maxSpeed: Float = 120f,  // Maximum speed in km/h
    val minSpeed: Float = 0f,    // Minimum speed in km/h
    val accelerationFactor: Float = 1.0f,  // Multiplier for acceleration/deceleration
    val engineTempFactor: Float = 1.0f,    // Multiplier for engine temperature
    val fuelConsumptionFactor: Float = 1.0f, // Multiplier for fuel consumption
    val batteryVoltageFactor: Float = 1.0f,  // Multiplier for battery voltage
    val oilPressureFactor: Float = 1.0f,     // Multiplier for oil pressure
    val trafficDensity: Float = 0.5f,        // 0.0 - 1.0, affects stop-and-go behavior
    val roadCondition: Float = 0.8f          // 0.0 - 1.0, affects smoothness
)

/**
 * Service that simulates various driving scenarios
 */
@Singleton
class DrivingScenarioSimulator @Inject constructor(
    private val obdService: OBDService
) {
    // Current scenario being simulated
    private val _activeScenario = MutableStateFlow<DrivingScenarioType?>(null)
    val activeScenario: StateFlow<DrivingScenarioType?> = _activeScenario.asStateFlow()
    
    // Status message for scenario simulation
    private val _statusMessage = MutableStateFlow<String?>(null)
    val statusMessage: StateFlow<String?> = _statusMessage.asStateFlow()
    
    // Time remaining in the current scenario (in milliseconds)
    private val _timeRemaining = MutableStateFlow(0L)
    val timeRemaining: StateFlow<Long> = _timeRemaining.asStateFlow()
    
    // Flag indicating if a scenario is currently running
    private val _isRunning = MutableStateFlow(false)
    val isRunning: StateFlow<Boolean> = _isRunning.asStateFlow()
    
    // Current speed target for the simulation
    private var speedTarget = 0f
    
    // Current scenario parameters
    private var scenarioParams = DrivingScenarioParams()
    
    // Coroutine scope for simulator operations
    private val simulationScope = CoroutineScope(Dispatchers.Default)
    
    /**
     * Start simulating a driving scenario
     * @param scenarioType The type of scenario to simulate
     * @param customParams Optional custom parameters to override defaults
     * @return True if scenario started successfully
     */
    fun startScenario(
        scenarioType: DrivingScenarioType, 
        customParams: DrivingScenarioParams? = null
    ): Boolean {
        if (_isRunning.value) {
            stopScenario()
        }
        
        // Set up scenario parameters
        scenarioParams = customParams ?: getDefaultParams(scenarioType)
        
        _activeScenario.value = scenarioType
        _statusMessage.value = "Starting ${scenarioType.name.lowercase().replace('_', ' ')} scenario"
        _isRunning.value = true
        _timeRemaining.value = scenarioParams.duration
        
        // Start the scenario simulation
        simulationScope.launch {
            try {
                simulateScenario(scenarioType, scenarioParams)
            } catch (e: Exception) {
                _statusMessage.value = "Scenario simulation error: ${e.message}"
            } finally {
                // Cleanup when simulation ends
                _isRunning.value = false
                _activeScenario.value = null
                _timeRemaining.value = 0
                _statusMessage.value = "Scenario ended"
                
                // Clear message after 3 seconds
                delay(3000)
                _statusMessage.value = null
            }
        }
        
        return true
    }
    
    /**
     * Stop the current scenario simulation
     */
    fun stopScenario() {
        if (!_isRunning.value) return
        
        _statusMessage.value = "Stopping scenario"
        _isRunning.value = false
    }
    
    /**
     * Get default parameters for a scenario type
     */
    private fun getDefaultParams(scenarioType: DrivingScenarioType): DrivingScenarioParams {
        return when (scenarioType) {
            DrivingScenarioType.CITY_DRIVING -> DrivingScenarioParams(
                maxSpeed = 60f,
                accelerationFactor = 0.8f,
                trafficDensity = 0.7f
            )
            DrivingScenarioType.HIGHWAY_DRIVING -> DrivingScenarioParams(
                minSpeed = 70f,
                maxSpeed = 130f,
                accelerationFactor = 0.6f,
                trafficDensity = 0.4f,
                roadCondition = 0.9f
            )
            DrivingScenarioType.MOUNTAIN_DRIVING -> DrivingScenarioParams(
                maxSpeed = 80f,
                accelerationFactor = 1.2f,
                engineTempFactor = 1.3f,
                fuelConsumptionFactor = 1.5f,
                roadCondition = 0.6f
            )
            DrivingScenarioType.AGGRESSIVE_DRIVING -> DrivingScenarioParams(
                maxSpeed = 140f,
                accelerationFactor = 2.0f,
                engineTempFactor = 1.5f,
                fuelConsumptionFactor = 1.8f,
                roadCondition = 0.7f
            )
            DrivingScenarioType.ENGINE_OVERHEAT -> DrivingScenarioParams(
                engineTempFactor = 2.5f,
                maxSpeed = 100f
            )
            DrivingScenarioType.LOW_FUEL -> DrivingScenarioParams(
                fuelConsumptionFactor = 3.0f
            )
            DrivingScenarioType.LOW_BATTERY -> DrivingScenarioParams(
                batteryVoltageFactor = 0.5f
            )
            DrivingScenarioType.NORMAL_DRIVING -> DrivingScenarioParams()
        }
    }
    
    /**
     * Simulate the actual scenario
     */
    private suspend fun simulateScenario(
        scenarioType: DrivingScenarioType,
        params: DrivingScenarioParams
    ) {
        val startTime = System.currentTimeMillis()
        val endTime = startTime + params.duration
        
        // Initial speed target based on scenario
        speedTarget = when (scenarioType) {
            DrivingScenarioType.CITY_DRIVING -> 30f
            DrivingScenarioType.HIGHWAY_DRIVING -> 100f
            DrivingScenarioType.MOUNTAIN_DRIVING -> 50f
            DrivingScenarioType.AGGRESSIVE_DRIVING -> 0f  // Will accelerate rapidly
            else -> 0f
        }
        
        // Scenario loop
        while (_isRunning.value && System.currentTimeMillis() < endTime) {
            // Update time remaining
            _timeRemaining.value = endTime - System.currentTimeMillis()
            
            // Progress factor (0.0 to 1.0) through the scenario
            val progress = 1f - (_timeRemaining.value / params.duration.toFloat())
            
            // Apply scenario-specific logic
            when (scenarioType) {
                DrivingScenarioType.CITY_DRIVING -> simulateCityDriving(progress)
                DrivingScenarioType.HIGHWAY_DRIVING -> simulateHighwayDriving(progress)
                DrivingScenarioType.MOUNTAIN_DRIVING -> simulateMountainDriving(progress)
                DrivingScenarioType.AGGRESSIVE_DRIVING -> simulateAggressiveDriving(progress)
                DrivingScenarioType.ENGINE_OVERHEAT -> simulateEngineOverheat(progress)
                DrivingScenarioType.LOW_FUEL -> simulateLowFuel(progress)
                DrivingScenarioType.LOW_BATTERY -> simulateLowBattery(progress)
                DrivingScenarioType.NORMAL_DRIVING -> simulateNormalDriving(progress)
            }
            
            delay(500) // Simulation update interval
        }
    }
    
    /**
     * City driving simulation with frequent speed changes and traffic stops
     */
    private suspend fun simulateCityDriving(progress: Float) {
        // Randomly change speed targets to simulate traffic lights and intersections
        if (Random.nextFloat() < 0.1) { // 10% chance each cycle to change speed
            speedTarget = if (Random.nextFloat() < scenarioParams.trafficDensity) {
                // More stops in heavy traffic
                0f
            } else {
                Random.nextFloat() * scenarioParams.maxSpeed
            }
        }
        
        // Apply commands to OBD service
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Highway driving with higher steady speeds
     */
    private suspend fun simulateHighwayDriving(progress: Float) {
        // Highway speeds with occasional slowing
        if (Random.nextFloat() < 0.05) { // 5% chance each cycle
            speedTarget = if (Random.nextFloat() < scenarioParams.trafficDensity) {
                // Slow for traffic
                scenarioParams.minSpeed + (Random.nextFloat() * 20f)
            } else {
                // Normal highway speed
                scenarioParams.minSpeed + 
                    (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
            }
        }
        
        // Apply commands to OBD service
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Mountain driving with varying speeds and increased engine load
     */
    private suspend fun simulateMountainDriving(progress: Float) {
        // Continuously varying speed to simulate uphill/downhill
        if (Random.nextFloat() < 0.15) { // 15% chance each cycle
            speedTarget = scenarioParams.minSpeed + 
                (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
        }
        
        // Increase engine temperature during uphill sections
        val tempFactor = 1.0f + (Math.sin(progress * Math.PI * 4).toFloat() * 0.5f)
        
        // Apply commands to OBD service
        obdService.setSimulatedSpeed(speedTarget)
        obdService.adjustSetting(OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR, 
            scenarioParams.engineTempFactor * tempFactor)
    }
    
    /**
     * Aggressive driving with rapid acceleration and deceleration
     */
    private suspend fun simulateAggressiveDriving(progress: Float) {
        // Dramatically change speeds frequently
        if (Random.nextFloat() < 0.2) { // 20% chance each cycle
            // Choose between fast acceleration or hard braking
            speedTarget = if (Random.nextFloat() < 0.5f) {
                // Fast acceleration to high speed
                scenarioParams.maxSpeed * (0.7f + Random.nextFloat() * 0.3f)
            } else {
                // Hard braking
                scenarioParams.minSpeed + (Random.nextFloat() * 20f)
            }
        }
        
        // Apply commands to OBD service
        obdService.setSimulatedSpeed(speedTarget)
        obdService.adjustSetting(OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR, 1.5f)
    }
    
    /**
     * Engine overheat scenario
     */
    private suspend fun simulateEngineOverheat(progress: Float) {
        // Gradually increase engine temperature
        val tempFactor = 1.0f + progress * scenarioParams.engineTempFactor
        
        // Apply commands to OBD service
        obdService.adjustSetting(OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR, tempFactor)
        
        // Normal driving otherwise
        if (Random.nextFloat() < 0.1) { // 10% chance each cycle
            speedTarget = scenarioParams.minSpeed + 
                (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
        }
        
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Low fuel scenario
     */
    private suspend fun simulateLowFuel(progress: Float) {
        // Gradually decrease fuel level
        obdService.adjustSetting(
            OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR, 
            scenarioParams.fuelConsumptionFactor
        )
        
        // Normal driving otherwise
        if (Random.nextFloat() < 0.1) { // 10% chance each cycle
            speedTarget = scenarioParams.minSpeed + 
                (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
        }
        
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Low battery scenario
     */
    private suspend fun simulateLowBattery(progress: Float) {
        // Gradually decrease battery voltage
        val voltageFactor = 1.0f - (progress * (1.0f - scenarioParams.batteryVoltageFactor))
        
        // Apply commands to OBD service
        obdService.adjustSetting(OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR, voltageFactor)
        
        // Normal driving otherwise
        if (Random.nextFloat() < 0.1) { // 10% chance each cycle
            speedTarget = scenarioParams.minSpeed + 
                (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
        }
        
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Normal balanced driving
     */
    private suspend fun simulateNormalDriving(progress: Float) {
        // Gradually change speeds in a natural way
        if (Random.nextFloat() < 0.08) { // 8% chance each cycle
            speedTarget = scenarioParams.minSpeed + 
                (Random.nextFloat() * (scenarioParams.maxSpeed - scenarioParams.minSpeed))
        }
        
        // Apply commands to OBD service
        obdService.setSimulatedSpeed(speedTarget)
    }
    
    /**
     * Get a list of available scenarios with descriptions
     * @return List of scenario names and descriptions
     */
    fun getAvailableScenarios(): List<Pair<String, String>> {
        return listOf(
            "city" to "City Driving - Start-stop traffic, moderate speeds",
            "highway" to "Highway Driving - Higher speeds, steady RPM",
            "mountain" to "Mountain Driving - Varying speeds, high RPM, temperature fluctuations",
            "aggressive" to "Aggressive Driving - Rapid acceleration/braking, high RPM",
            "overheat" to "Engine Overheat - Gradual engine temperature increase",
            "lowfuel" to "Low Fuel - Gradual fuel depletion",
            "lowbattery" to "Low Battery - Battery voltage drop scenario",
            "normal" to "Normal Driving - Balanced, typical driving pattern"
        )
    }
    
    /**
     * Start a scenario by name
     * @param name The name of the scenario to start
     * @return True if scenario started successfully
     */
    fun startScenarioByName(name: String): Boolean {
        val scenarioType = when (name.lowercase()) {
            "city" -> DrivingScenarioType.CITY_DRIVING
            "highway" -> DrivingScenarioType.HIGHWAY_DRIVING
            "mountain" -> DrivingScenarioType.MOUNTAIN_DRIVING
            "aggressive" -> DrivingScenarioType.AGGRESSIVE_DRIVING
            "overheat" -> DrivingScenarioType.ENGINE_OVERHEAT
            "lowfuel" -> DrivingScenarioType.LOW_FUEL
            "lowbattery" -> DrivingScenarioType.LOW_BATTERY
            "normal" -> DrivingScenarioType.NORMAL_DRIVING
            else -> return false
        }
        
        return startScenario(scenarioType)
    }
} 