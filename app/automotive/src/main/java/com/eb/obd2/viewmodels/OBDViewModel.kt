package com.eb.obd2.viewmodels

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.eb.obd2.models.AnimationCurve
import com.eb.obd2.models.ConnectionStatus
import com.eb.obd2.models.DrivingProfile
import com.eb.obd2.models.DrivingState
import com.eb.obd2.models.ProfileCategory
import com.eb.obd2.models.RenderStatus
import com.eb.obd2.models.RuntimeRecord
import com.eb.obd2.repositories.AnimationCurveRepository
import com.eb.obd2.repositories.ProfileRepository
import com.eb.obd2.repositories.RecordRepository
import com.eb.obd2.services.OBDConnectionFactory
import com.eb.obd2.services.OBDService
import com.eb.obd2.services.DataRecordingService
import com.eb.obd2.views.components.SystemStatus
import com.eb.obd2.views.components.VehicleWarning
import com.eb.obd2.views.components.WarningLevel
import com.github.pires.obd.commands.SpeedCommand
import com.github.pires.obd.commands.engine.OilTempCommand
import com.github.pires.obd.commands.engine.RPMCommand
import com.github.pires.obd.commands.fuel.FuelLevelCommand
import com.github.pires.obd.commands.engine.ThrottlePositionCommand
import com.patrykandpatrick.vico.core.cartesian.data.CartesianChartModelProducer
import com.patrykandpatrick.vico.core.cartesian.data.lineSeries
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.launch
import java.time.Duration
import java.time.LocalDateTime
import java.util.UUID
import javax.inject.Inject
import kotlin.math.absoluteValue
import kotlin.random.Random
import kotlinx.serialization.Serializable
import kotlinx.coroutines.withContext

// Add CommandStatus sealed class
sealed class CommandStatus {
    object Idle : CommandStatus()
    object Sending : CommandStatus()
    data class Success(val result: String = "") : CommandStatus()
    data class Error(val message: String = "") : CommandStatus()
}

@HiltViewModel
class OBDViewModel @Inject constructor(
    private val obdService: OBDService,
    private val factory: OBDConnectionFactory,
    private val repository: RecordRepository,
    private val curveRepository: AnimationCurveRepository,
    private val profileRepository: ProfileRepository,
    private val dataRecordingService: DataRecordingService
) : ViewModel() {

    /** The status of view model. */
    var status: RenderStatus by mutableStateOf(RenderStatus.LOADING)
        private set

    /** The FSM driving state */
    var drivingState: DrivingState by mutableStateOf(DrivingState.IDLE)
        private set

    /** The comfort level [0.0, 1.0] 0 is really bad, 1.0 is comfortable */
    var comfortLevel: Float by mutableFloatStateOf(1.0f)
        private set

    /** The latest acceleration */
    var acceleration: Float by mutableFloatStateOf(0.0f)
        private set
        
    /** The current RPM value */
    var rpm: Float by mutableFloatStateOf(0.0f)
        private set
        
    /** The current engine temperature value in Celsius */
    var engineTemperature: Float by mutableFloatStateOf(70.0f)
        private set
        
    /** The current fuel level as percentage */
    var fuelLevel: Float by mutableFloatStateOf(100.0f)
        private set
        
    /** The current fuel consumption rate in L/100km */
    var fuelConsumptionRate: Float by mutableFloatStateOf(0.0f)
        private set
        
    /** Vehicle speed in km/h */
    var speed: Float by mutableFloatStateOf(0.0f)
        private set
        
    /** The current gear (1-6) */
    var currentGear: Int by mutableIntStateOf(1)
        private set
        
    /** The current gear position (P, R, N, D) */
    var gearPosition: String by mutableStateOf("P")
        private set
        
    /** The current throttle position as percentage */
    var throttlePosition: Float by mutableFloatStateOf(0.0f)
        private set
        
    /** The current brake position as percentage */
    var brakePosition: Float by mutableFloatStateOf(0.0f)
        private set
    
    /** The average fuel consumption in L/100km */
    var averageFuelConsumption: Float by mutableFloatStateOf(8.0f)
        private set
    
    /** The estimated range in kilometers */
    var estimatedRange: Float by mutableFloatStateOf(450.0f)
        private set
    
    /** Battery voltage in volts */
    var batteryVoltage: Float by mutableFloatStateOf(12.6f)
        private set
    
    /** Oil pressure in PSI */
    var oilPressure: Float by mutableFloatStateOf(30.0f)
        private set

    /** Enhanced driving style metrics */
    
    /** Acceleration behavior score [0.0, 1.0] */
    var accelerationScore: Float by mutableFloatStateOf(1.0f)
        private set
        
    /** Deceleration/braking behavior score [0.0, 1.0] */
    var decelerationScore: Float by mutableFloatStateOf(1.0f)
        private set
        
    /** Cornering stability score [0.0, 1.0] */
    var corneringScore: Float by mutableFloatStateOf(1.0f)
        private set
        
    /** Engine efficiency score [0.0, 1.0] */
    var engineEfficiencyScore: Float by mutableFloatStateOf(1.0f)
        private set
        
    /** Overall driving efficiency score [0.0, 1.0] */
    var efficiencyScore: Float by mutableFloatStateOf(0.8f)
        private set
        
    /** Overall driving smoothness score [0.0, 1.0] */
    var smoothnessScore: Float by mutableFloatStateOf(0.9f)
        private set
        
    /** Overall driving score [0.0, 100.0] */
    var overallDrivingScore: Float by mutableFloatStateOf(85.0f)
        private set
        
    /** Vehicle systems status */
    
    /** Engine system status */
    var engineSystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Transmission system status */
    var transmissionSystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Fuel system status */
    var fuelSystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Emission system status */
    var emissionSystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Brake system status */
    var brakeSystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Battery status */
    var batterySystemStatus: SystemStatus by mutableStateOf(SystemStatus.NORMAL)
        private set
        
    /** Engine system status details */
    var engineSystemDetails: String by mutableStateOf("")
        private set
        
    /** Transmission system status details */
    var transmissionSystemDetails: String by mutableStateOf("")
        private set
        
    /** Fuel system status details */
    var fuelSystemDetails: String by mutableStateOf("")
        private set
        
    /** Emission system status details */
    var emissionSystemDetails: String by mutableStateOf("")
        private set
        
    /** Brake system status details */
    var brakeSystemDetails: String by mutableStateOf("")
        private set
        
    /** Battery status details */
    var batterySystemDetails: String by mutableStateOf("")
        private set
        
    /** Active warning notifications */
    private val _activeWarnings = mutableStateListOf<VehicleWarning>()
    val activeWarnings: List<VehicleWarning> get() = _activeWarnings.toList()
    
    /** Set to track dismissed warning IDs */
    private val dismissedWarningIds = mutableSetOf<String>()

    /** The pid key with record map. */
    val obdData = mutableStateMapOf<String, MutableList<RuntimeRecord>>()

    /** The aggressive score list */
    val aggressiveScores = mutableStateListOf(0.0f)
    
    /** Historical engine temperature data */
    val temperatureHistory = mutableStateListOf<Pair<Long, Float>>()
    
    /** Historical fuel consumption data */
    val fuelConsumptionHistory = mutableStateListOf<Pair<Long, Float>>()
    
    /** Historical fuel level data */
    val fuelLevelHistory = mutableStateListOf<Pair<Long, Float>>()
    
    /** Historical speed data */
    val speedHistory = mutableStateListOf<Pair<Long, Float>>()
    
    /** The chart model producer */
    val modelProducer = CartesianChartModelProducer.build()
    
    /** The temperature chart model producer */
    val temperatureModelProducer = CartesianChartModelProducer.build()
    
    /** The fuel consumption chart model producer */
    val consumptionModelProducer = CartesianChartModelProducer.build()
    
    /** The fuel level chart model producer */
    val fuelLevelModelProducer = CartesianChartModelProducer.build()
    
    /** The speed chart model producer */
    val speedModelProducer = CartesianChartModelProducer.build()

    // Animation and curve properties
    private var _animationSpeed: Float by mutableFloatStateOf(1.0f)
    val animationSpeed: Float get() = _animationSpeed
    
    var isRecordingPattern: Boolean by mutableStateOf(false)
        private set
        
    var isPlayingCurve: Boolean by mutableStateOf(false)
        private set
        
    var simulationEnabled: Boolean by mutableStateOf(false)
        private set
        
    var currentCurve: AnimationCurve? by mutableStateOf(null)
        private set
        
    var curveProgress: Float by mutableFloatStateOf(0f)
        private set
        
    var curveStatusMessage: String by mutableStateOf("")
        private set
        
    // Profile management properties 
    private val _localProfiles = mutableStateListOf<DrivingProfile>()
    val localProfiles: List<DrivingProfile> get() = _localProfiles.toList()
    
    private val _remoteProfiles = mutableStateListOf<DrivingProfile>()
    val remoteProfiles: List<DrivingProfile> get() = _remoteProfiles.toList()
    
    private val _filteredProfiles = mutableStateListOf<DrivingProfile>()
    val filteredProfiles: List<DrivingProfile> get() = _filteredProfiles.toList()
    
    var _savedCurves = mutableStateListOf<AnimationCurve>()
    val savedCurves: List<AnimationCurve> get() = _savedCurves.toList()
    
    var selectedCategory: ProfileCategory by mutableStateOf(ProfileCategory.GENERAL)
        private set
        
    var activeProfile: DrivingProfile? by mutableStateOf(null)
        private set
        
    var profileStatusMessage: String by mutableStateOf("")
        private set
        
    var isSyncingProfiles: Boolean by mutableStateOf(false)
        private set

    private val maxBufferCapacity = 32
    private val maxHistoryCapacity = 100

    private val aggressiveWeight = 0.9f
    private val nonAggressiveWeight = 0.975f
    
    private val smoothnessWeight = 0.8f
    private val efficiencyWeight = 0.7f

    private var drivingStateStartTime: LocalDateTime? = null
    
    // Recorded speed samples for pattern recording (timestamp, speed)
    private val speedSamples = mutableListOf<Pair<Long, Float>>()
    private var recordingStartTime: Long = 0L
    
    // Custom PID values for gear position and current gear
    private val PID_GEAR_POSITION = "A1" // Custom PID for gear position
    private val PID_CURRENT_GEAR = "A2"  // Custom PID for current gear
    
    // Custom PID values for simulation controls
    private val PID_THROTTLE = "B1" // Custom PID for throttle position
    private val PID_BRAKE = "B2"    // Custom PID for brake position
    private val PID_ANIM_SPEED = "B3" // Custom PID for animation speed
    
    // Custom PID values for animation curves
    private val PID_CURVE_DATA = "C1" // Custom PID for curve data
    private val PID_CURVE_CONTROL = "C2" // Custom PID for curve control
    
    // Custom PID values for profiles
    private val PID_PROFILE_DATA = "D1" // Custom PID for profile data
    private val PID_PROFILE_CONTROL = "D2" // Custom PID for profile control

    /** Command status from OBD service */
    val commandStatus = obdService.commandStatus
    
    /** Available emulator settings with current values */
    private val _emulatorSettings = MutableStateFlow<Map<OBDService.EmulatorSetting, Float>>(emptyMap())
    val emulatorSettings: StateFlow<Map<OBDService.EmulatorSetting, Float>> = _emulatorSettings.asStateFlow()
    
    /** Status message for emulator commands */
    private val _commandStatusMessage = MutableStateFlow<String?>(null)
    val commandStatusMessage: StateFlow<String?> = _commandStatusMessage.asStateFlow()
    
    /** Two-way communication enabled flag */
    private val _twoWayCommEnabled = MutableStateFlow(true)
    val twoWayCommEnabled: StateFlow<Boolean> = _twoWayCommEnabled.asStateFlow()

    /** Connection state from OBD service */
    val connectionState = obdService.connectionStatus

    init {
        viewModelScope.launch {
            // Register additional commands for new data points
            obdService.registerCommand(
                OilTempCommand(),
                FuelLevelCommand(),
                FuelLevelCommand(),
                RPMCommand(),
                ThrottlePositionCommand()
            )
            
            obdService.startConnection()
            
            launch {
                obdService.vehicleData.collect { vehicleData ->
                    // Update UI state with vehicle data
                    rpm = vehicleData.rpm
                    speed = vehicleData.speed
                    engineTemperature = vehicleData.engineTemp
                    currentGear = vehicleData.gear
                    gearPosition = vehicleData.gearPosition
                    throttlePosition = vehicleData.throttlePercentage
                    brakePosition = vehicleData.brakePercentage
                    fuelLevel = vehicleData.fuelLevel
                    fuelConsumptionRate = vehicleData.fuelConsumptionRate
                    
                    // Analyze driving style
                    analyseStyle(vehicleData.speed, calculateAcceleration(vehicleData.speed))
                    
                    // Update histories
                    addToSpeedHistory(vehicleData.speed)
                    addToTemperatureHistory(vehicleData.engineTemp)
                    addToFuelLevelHistory(vehicleData.fuelLevel)
                }
            }

            launch {
                connectionState.collect { state ->
                    status = when (state) {
                        ConnectionStatus.CONNECTING -> RenderStatus.LOADING
                        ConnectionStatus.CONNECTED -> RenderStatus.SUCCESS
                        ConnectionStatus.DISCONNECTED -> RenderStatus.ERROR
                        ConnectionStatus.FAILED -> RenderStatus.ERROR
                        else -> RenderStatus.ERROR
                    }
                }
            }
        }
        
        // Initialize default emulator settings
        _emulatorSettings.value = mapOf(
            OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR to 1.0f,
            OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR to 1.0f,
            OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR to 1.0f,
            OBDService.EmulatorSetting.SIMULATION_SPEED to 1.0f,
            OBDService.EmulatorSetting.FAULT_SIMULATION to 0.0f
        )
        
        viewModelScope.launch {
            // Monitor command status
            launch {
                obdService.commandStatus.collect { status ->
                    when (status) {
                        is CommandStatus.Error -> {
                            _commandStatusMessage.value = "Command failed: ${status.message}"
                            // Auto-clear message after 3 seconds
                            delay(3000)
                            _commandStatusMessage.value = null
                        }
                        is CommandStatus.Success -> {
                            _commandStatusMessage.value = "Command executed successfully"
                            // Auto-clear message after 2 seconds
                            delay(2000)
                            _commandStatusMessage.value = null
                        }
                        else -> {
                            // No need to show Idle or Sending messages
                        }
                    }
                }
            }
        }
        
        // Start data simulation for development
        simulateOBDData()
    }
    
    /**
     * Get the average fuel consumption (over last 100km or session)
     * This is a simulated value based on current consumption with some variance
     */
    fun getAverageConsumption(): Float {
        // Add some randomness around the current consumption rate to simulate a realistic average
        val variance = fuelConsumptionRate * 0.15f // 15% variance
        return (fuelConsumptionRate + Random.nextFloat() * variance - variance / 2)
            .coerceIn(5.0f, 15.0f) // Keep within reasonable bounds
    }
    
    /**
     * Calculate the estimated range based on current fuel level and consumption
     * @return Estimated range in kilometers
     */
    fun calculateEstimatedRange(): Float {
        // Simple calculation: fuel level percentage * tank capacity (50L) / consumption per 100km * 100
        val tankCapacity = 50.0f // Liters
        val remainingFuel = tankCapacity * (fuelLevel / 100.0f)
        val avgConsumption = getAverageConsumption()
        
        // Avoid division by zero
        return if (avgConsumption > 0) {
            (remainingFuel / avgConsumption) * 100.0f
        } else {
            0.0f
        }
    }
    
    /**
     * Update throttle position
     */
    fun updateThrottlePosition(position: Float) {
        val safePosition = position.coerceIn(0.0f, 1.0f)
        throttlePosition = safePosition
        
        // Send command to server if two-way communication is enabled
        if (_twoWayCommEnabled.value) {
            obdService.setThrottlePosition(safePosition)
        }
    }
    
    /**
     * Update brake position
     */
    fun updateBrakePosition(position: Float) {
        val safePosition = position.coerceIn(0.0f, 1.0f)
        brakePosition = safePosition
        
        // Send command to server if two-way communication is enabled
        if (_twoWayCommEnabled.value) {
            obdService.setBrakePosition(safePosition)
        }
    }
    
    /**
     * Update animation speed
     */
    fun updateAnimationSpeed(speed: Float) {
        val safeSpeed = speed.coerceIn(0.1f, 2.0f)
        _animationSpeed = safeSpeed
        
        // Send command to server if two-way communication is enabled
        if (_twoWayCommEnabled.value) {
            obdService.adjustSetting(OBDService.EmulatorSetting.SIMULATION_SPEED, safeSpeed)
        }
    }
    
    /**
     * Start recording a driving pattern
     */
    fun startRecordingPattern() {
        if (isRecordingPattern || !simulationEnabled) return
        
        isRecordingPattern = true
        recordingStartTime = System.currentTimeMillis()
        speedSamples.clear()
        
        curveStatusMessage = "Recording driving pattern..."
    }
    
    /**
     * Stop recording and save the driving pattern
     */
    fun stopRecordingPattern(name: String = "Recorded Pattern", description: String = "") {
        if (!isRecordingPattern) return
        
        isRecordingPattern = false
        curveStatusMessage = "Processing recorded pattern..."
        
        viewModelScope.launch {
            try {
                // Create animation curve from recorded data
                val curve = curveRepository.recordDrivingPattern(
                    speedSamples = speedSamples.toList(),
                    name = name,
                    description = description
                )
                
                // Save the curve
                val success = curveRepository.saveCurve(curve)
                
                curveStatusMessage = if (success) {
                    "Pattern saved successfully"
                } else {
                    "Failed to save pattern"
                }
                
                // Clear message after 3 seconds
                delay(3000)
                curveStatusMessage = ""
            } catch (e: Exception) {
                e.printStackTrace()
                curveStatusMessage = "Error: ${e.message}"
                
                // Clear message after 3 seconds
                delay(3000)
                curveStatusMessage = ""
            }
        }
    }
    
    /**
     * Start playing an animation curve
     */
    fun playCurve(curve: AnimationCurve) {
        if (isPlayingCurve || !simulationEnabled) return
        
        currentCurve = curve
        isPlayingCurve = true
        curveProgress = 0.0f
        
        curveStatusMessage = "Playing curve: ${curve.name}"
        
        // Send curve to server
        sendCurveToServer(curve)
        
        // Simulate curve playback locally
        viewModelScope.launch {
            try {
                // Calculate total duration
                val duration = curve.duration()
                
                if (duration <= 0f) {
                    stopCurvePlayback()
                    return@launch
                }
                
                val startTime = System.currentTimeMillis()
                
                while (isPlayingCurve) {
                    val elapsed = System.currentTimeMillis() - startTime
                    val adjustedElapsed = elapsed * _animationSpeed
                    
                    curveProgress = (adjustedElapsed / duration).coerceIn(0f, 1f)
                    
                    if (curveProgress >= 1.0f) {
                        stopCurvePlayback()
                        break
                    }
                    
                    delay(16) // ~60fps
                }
            } catch (e: Exception) {
                e.printStackTrace()
                stopCurvePlayback()
            }
        }
    }
    
    /**
     * Stop playing the current animation curve
     */
    fun stopCurvePlayback() {
        if (!isPlayingCurve) return
        
        isPlayingCurve = false
        curveProgress = 0.0f
        
        // Send stop command to server
        sendCurveControlCommand("stop")
        
        curveStatusMessage = "Curve playback stopped"
        
        // Clear message after 3 seconds
        viewModelScope.launch {
            delay(3000)
            if (!isPlayingCurve && !isRecordingPattern) {
                curveStatusMessage = ""
            }
        }
    }
    
    /**
     * Delete a saved animation curve
     */
    fun deleteCurve(curve: AnimationCurve) {
        viewModelScope.launch {
            curveRepository.deleteCurve(curve)
        }
    }
    
    /**
     * Set the profile category filter for browsing
     */
    fun setProfileCategoryFilter(category: ProfileCategory) {
        selectedCategory = category
        updateFilteredProfiles()
    }
    
    /**
     * Update the filtered profiles list based on selected category
     */
    private fun updateFilteredProfiles() {
        viewModelScope.launch(Dispatchers.Main) {
            if (selectedCategory == ProfileCategory.GENERAL) {
                // Show all profiles
                _filteredProfiles.clear()
                _filteredProfiles.addAll(localProfiles + remoteProfiles)
            } else {
                // Filter by selected category
                _filteredProfiles.clear()
                val filteredList = (localProfiles + remoteProfiles).filter { profile ->
                    profile.category == selectedCategory
                }
                _filteredProfiles.addAll(filteredList)
            }
        }
    }
    
    /**
     * Sync profiles with server
     */
    fun syncProfilesWithServer() {
        viewModelScope.launch {
            profileStatusMessage = "Syncing with server..."
            val success = profileRepository.syncWithServer()
            
            profileStatusMessage = if (success) {
                "Sync completed"
            } else {
                "Sync failed"
            }
            
            // Clear message after 3 seconds
            delay(3000)
            profileStatusMessage = ""
        }
    }
    
    /**
     * Activate a driving profile (load its settings and curves)
     */
    fun activateProfile(profile: DrivingProfile) {
        activeProfile = profile
        
        // Apply profile settings
        profile.defaultSettings["animation_speed"]?.let {
            updateAnimationSpeed(it)
        }
        
        profileStatusMessage = "Profile activated: ${profile.name}"
        
        // Auto-play first curve if profile has curves
        if (profile.animationCurves.isNotEmpty() && !isPlayingCurve) {
            profile.getCurve(0)?.let { curve ->
                playCurve(curve)
            }
        }
        
        // Clear message after 3 seconds
        viewModelScope.launch {
            delay(3000)
            profileStatusMessage = ""
        }
    }
    
    /**
     * Save a profile with the current settings
     */
    fun saveCurrentProfile(name: String, description: String, category: ProfileCategory) {
        viewModelScope.launch {
            profileStatusMessage = "Saving profile..."
            
            try {
                // Get current settings
                val settings = mapOf(
                    "animation_speed" to _animationSpeed
                )
                
                // Create profile with all currently saved curves
                val profile = profileRepository.createProfileFromCurves(
                    name = name,
                    description = description,
                    category = category,
                    curves = _savedCurves,
                    settings = settings
                )
                
                profileStatusMessage = "Profile saved: ${profile.name}"
                
                // Automatically activate the new profile
                activateProfile(profile)
                
                // Clear message after 3 seconds
                delay(3000)
                profileStatusMessage = ""
            } catch (e: Exception) {
                e.printStackTrace()
                profileStatusMessage = "Error saving profile: ${e.message}"
                
                // Clear message after 3 seconds
                delay(3000)
                profileStatusMessage = ""
            }
        }
    }
    
    /**
     * Delete a profile
     */
    fun deleteProfile(profile: DrivingProfile) {
        viewModelScope.launch {
            profileStatusMessage = "Deleting profile..."
            
            val success = profileRepository.deleteProfile(profile)
            
            profileStatusMessage = if (success) {
                // If we're deleting the active profile, clear it
                if (activeProfile?.id == profile.id) {
                    activeProfile = null
                }
                
                "Profile deleted"
            } else {
                "Failed to delete profile"
            }
            
            // Clear message after 3 seconds
            delay(3000)
            profileStatusMessage = ""
        }
    }
    
    /**
     * Import a profile from server to local storage
     */
    fun importProfileFromServer(profile: DrivingProfile) {
        if (!profile.isRemote) return
        
        viewModelScope.launch {
            profileStatusMessage = "Importing profile..."
            
            val success = profileRepository.importProfileFromServer(profile)
            
            profileStatusMessage = if (success) {
                "Profile imported"
            } else {
                "Failed to import profile"
            }
            
            // Clear message after 3 seconds
            delay(3000)
            profileStatusMessage = ""
        }
    }
    
    /**
     * Upload a local profile to the server
     */
    fun uploadProfileToServer(profile: DrivingProfile) {
        if (profile.isRemote) return
        
        viewModelScope.launch {
            profileStatusMessage = "Uploading profile..."
            
            val success = profileRepository.uploadProfileToServer(profile)
            
            profileStatusMessage = if (success) {
                "Profile uploaded to server"
            } else {
                "Failed to upload profile"
            }
            
            // Clear message after 3 seconds
            delay(3000)
            profileStatusMessage = ""
        }
    }
    
    /**
     * Record a speed sample for pattern recording
     */
    private fun recordSpeedSample(speed: Float) {
        val currentTime = System.currentTimeMillis()
        speedSamples.add(Pair(currentTime, speed))
    }
    
    /**
     * Send an animation curve to the server
     */
    private fun sendCurveToServer(curve: AnimationCurve) {
        // Convert curve to a format that can be sent to the server
        val jsonData = curve.toJson()
        
        // Extract points for sending in a format the server can understand
        val pointsData = curve.points.joinToString(",") { point ->
            "${point.x}:${point.y}"
        }
        
        // TODO: Implement actual sending mechanism
    }
    
    /**
     * Send a curve control command to the server
     */
    private fun sendCurveControlCommand(command: String) {
        // TODO: Implement actual sending mechanism
    }
    
    /**
     * Send a simulation command to the OBD server
     */
    private fun sendSimulationCommand(pid: String, value: Float) {
        // TODO: Implement actual command sending mechanism
        // This would communicate with the OBD emulator via a custom command
        
        // For now just update the UI state
        // In a real implementation, this would send a command through the OBD connection
    }
    
    /**
     * Updates the gear position based on speed and RPM
     * This matches the adapter's gear calculation logic
     */
    private fun updateGearPosition(speed: Float) {
        // In a real car, gear position would come from transmission control unit
        // Here we're simulating it based on speed:
        gearPosition = when {
            speed <= 0.1f -> "P" // Parked at very low speed
            speed < 0f -> "R"    // Negative speed = reverse
            speed < 2f -> "N"    // Neutral at very low forward speed
            else -> "D"          // Drive mode at normal speed
        }
        
        // Update current gear based on speed and RPM when in drive mode
        if (gearPosition == "D") {
            // Get current RPM from OBD data
            val currentRpm = obdData["rpm"]?.lastOrNull()?.value?.toFloat() ?: 0f
            
            // Calculate target gear based on speed
            val targetGear = when {
                speed < 20f -> 1
                speed < 40f -> 2
                speed < 60f -> 3
                speed < 80f -> 4
                speed < 100f -> 5
                else -> 6
            }
            
            // Consider RPM for gear changes (matching adapter thresholds)
            val upshiftRpm = 3000f
            val downshiftRpm = 1500f
            
            currentGear = when {
                // Don't upshift if RPM is too low
                currentRpm < downshiftRpm && currentGear > 1 -> currentGear - 1
                // Don't downshift if RPM is too high
                currentRpm > upshiftRpm && currentGear < 6 -> currentGear + 1
                // Otherwise use speed-based target gear
                else -> targetGear
            }
        } else {
            // Not in drive mode, gear is not applicable
            currentGear = 1
        }
    }

    override fun onCleared() {
        super.onCleared()
        obdService.stopConnection()
    }

    private fun shouldUpdateData(pidKey: String, newData: RuntimeRecord): Boolean {
        val lastRecord = obdData[pidKey]?.lastOrNull() ?: return true
        return lastRecord.value != newData.value
    }

    private fun analyseStyle(speed: Float, acceleration: Float) {
        val currentTime = LocalDateTime.now()
        
        // Determine if driving is smooth based on acceleration
        val isSmooth = acceleration.absoluteValue < 3.0f
        updateSmoothnessScore(isSmooth)

        when (drivingState) {
            DrivingState.IDLE -> {
                if (acceleration > 5.7f) {
                    drivingState = DrivingState.AGGRESSIVE_ACCELERATION
                    drivingStateStartTime = currentTime
                    comfortLevel = 1.0f
                }
            }
            DrivingState.NORMAL -> {
                if (speed == 0.0f) drivingState = DrivingState.IDLE
                comfortLevel = 1.0f
            }
            DrivingState.AGGRESSIVE_ACCELERATION -> {
                if (acceleration < -7.5f) {
                    val stateElapsed = Duration.between(drivingStateStartTime, currentTime).seconds
                    if (stateElapsed <= 3) comfortLevel = 0.0f
                    drivingState = DrivingState.NORMAL
                }
            }
            DrivingState.AGGRESSIVE_DECELERATION -> {
                drivingState = DrivingState.NORMAL
                comfortLevel = 0.0f
            }
        }

        val isAggressive = acceleration in -7.5f..5.7f

        calculateAggressive(if (isAggressive) 1.0f else 0.0f, isAggressive)
    }

    private fun calculateAggressive(acceleration: Float, isAggressive: Boolean) {
        // Always run on main thread when modifying Compose state
        viewModelScope.launch(Dispatchers.Main) {
            val weight = if (isAggressive) aggressiveWeight else nonAggressiveWeight

            if (aggressiveScores.size > maxBufferCapacity) {
                aggressiveScores.removeAt(0)
            }

            aggressiveScores.add(aggressiveScores.last() * weight + acceleration * (1.0f - weight))
        }
    }

    /**
     * Add a temperature data point to the history
     */
    private fun addToTemperatureHistory(temperature: Float) {
        viewModelScope.launch(Dispatchers.Main) {
            val timestamp = System.currentTimeMillis()
            temperatureHistory.add(timestamp to temperature)
            
            // Keep history within capacity
            if (temperatureHistory.size > maxHistoryCapacity) {
                temperatureHistory.removeAt(0)
            }
        }
    }
    
    /**
     * Add a fuel consumption data point to the history
     */
    private fun addToFuelConsumptionHistory(consumption: Float) {
        viewModelScope.launch(Dispatchers.Main) {
            val timestamp = System.currentTimeMillis()
            fuelConsumptionHistory.add(timestamp to consumption)
            
            // Keep history within capacity
            if (fuelConsumptionHistory.size > maxHistoryCapacity) {
                fuelConsumptionHistory.removeAt(0)
            }
        }
    }
    
    /**
     * Add a fuel level data point to the history
     */
    private fun addToFuelLevelHistory(level: Float) {
        viewModelScope.launch(Dispatchers.Main) {
            val timestamp = System.currentTimeMillis()
            fuelLevelHistory.add(timestamp to level)
            
            // Keep history within capacity
            if (fuelLevelHistory.size > maxHistoryCapacity) {
                fuelLevelHistory.removeAt(0)
            }
        }
    }
    
    /**
     * Add a speed data point to the history
     */
    private fun addToSpeedHistory(speed: Float) {
        viewModelScope.launch(Dispatchers.Main) {
            val timestamp = System.currentTimeMillis()
            speedHistory.add(timestamp to speed)
            
            // Keep history within capacity
            if (speedHistory.size > maxHistoryCapacity) {
                speedHistory.removeAt(0)
            }
        }
    }
    
    /**
     * Update the efficiency score based on fuel consumption
     */
    private fun updateEfficiencyScore(consumption: Float) {
        // Lower consumption is better
        // Assuming consumption range: 5 L/100km (good) to 15 L/100km (bad)
        val rawScore = 1.0f - ((consumption - 5.0f) / 10.0f).coerceIn(0.0f, 1.0f)
        
        // Apply smoothing
        efficiencyScore = (efficiencyScore * efficiencyWeight) + (rawScore * (1.0f - efficiencyWeight))
    }
    
    /**
     * Update the smoothness score based on acceleration patterns
     */
    private fun updateSmoothnessScore(isSmooth: Boolean) {
        val rawScore = if (isSmooth) 1.0f else 0.0f
        
        // Apply smoothing
        smoothnessScore = (smoothnessScore * smoothnessWeight) + (rawScore * (1.0f - smoothnessWeight))
    }
    
    /**
     * Calculate the overall driving score
     */
    private fun calculateOverallDrivingScore() {
        // Weighted combination of different factors
        // 1. Aggressiveness (lower is better)
        val aggressivenessFactor = 1.0f - aggressiveScores.last().coerceIn(0.0f, 1.0f)
        
        // 2. Fuel efficiency (higher is better)
        val efficiencyFactor = efficiencyScore
        
        // 3. Driving smoothness (higher is better)
        val smoothnessFactor = smoothnessScore
        
        // 4. Comfort level (higher is better)
        val comfortFactor = comfortLevel
        
        // Calculate weighted average (on a 0-100 scale)
        overallDrivingScore = (
            (aggressivenessFactor * 0.3f) +
            (efficiencyFactor * 0.3f) +
            (smoothnessFactor * 0.2f) +
            (comfortFactor * 0.2f)
        ) * 100.0f
    }
    
    /**
     * Update all chart models with the latest data
     */
    private fun updateChartModels() {
        // Update main chart (aggressive scores)
        modelProducer.tryRunTransaction {
            lineSeries {
                series(aggressiveScores)
            }
        }
        
        // Update temperature chart if we have data
        if (temperatureHistory.isNotEmpty()) {
            temperatureModelProducer.tryRunTransaction {
                lineSeries {
                    series(temperatureHistory.map { it.second })
                }
            }
        }
        
        // Update fuel consumption chart if we have data
        if (fuelConsumptionHistory.isNotEmpty()) {
            consumptionModelProducer.tryRunTransaction {
                lineSeries {
                    series(fuelConsumptionHistory.map { it.second })
                }
            }
        }
        
        // Update fuel level chart if we have data
        if (fuelLevelHistory.isNotEmpty()) {
            fuelLevelModelProducer.tryRunTransaction {
                lineSeries {
                    series(fuelLevelHistory.map { it.second })
                }
            }
        }
        
        // Update speed chart if we have data
        if (speedHistory.isNotEmpty()) {
            speedModelProducer.tryRunTransaction {
                lineSeries {
                    series(speedHistory.map { it.second })
                }
            }
        }
    }

    /** The statistics recording controls */
    
    /**
     * Start recording vehicle data
     */
    fun startDataRecording(name: String? = null) {
        dataRecordingService.startRecording(name)
    }
    
    /**
     * Stop recording vehicle data
     */
    fun stopDataRecording() {
        dataRecordingService.stopRecording()
    }
    
    /**
     * Get available recorded data sessions
     */
    fun getRecordedSessions(): StateFlow<List<String>> {
        return dataRecordingService.availableSessions
    }
    
    /**
     * Check if data recording is in progress
     */
    fun isRecording(): StateFlow<Boolean> {
        return dataRecordingService.isRecording
    }
    
    /**
     * Get current recording session name
     */
    fun getCurrentSessionName(): StateFlow<String?> {
        return dataRecordingService.sessionName
    }
    
    /**
     * Get current recording duration in seconds
     */
    fun getRecordingDuration(): StateFlow<Long> {
        return dataRecordingService.recordingDuration
    }
    
    /**
     * Enable or disable data recording
     */
    fun setDataRecordingEnabled(enabled: Boolean) {
        dataRecordingService.setEnabled(enabled)
    }
    
    /**
     * Delete a recorded session
     */
    fun deleteRecordedSession(name: String): Boolean {
        return dataRecordingService.deleteSession(name)
    }
    
    /**
     * Load a recorded session for analysis
     */
    fun loadRecordedSession(name: String): Map<String, List<DataRecordingService.DataPoint>> {
        return dataRecordingService.loadSession(name)
    }

    /**
     * Enable or disable two-way communication with the emulator
     */
    fun setTwoWayCommunicationEnabled(enabled: Boolean) {
        _twoWayCommEnabled.value = enabled
    }
    
    /**
     * Start a simulation profile on the emulator
     */
    fun startSimulationProfile(profileId: String, parameters: Map<String, String> = emptyMap()): Boolean {
        if (!_twoWayCommEnabled.value || profileId.isBlank()) return false
        return obdService.triggerSimulationProfile(profileId, parameters)
    }
    
    /**
     * Start a predefined driving scenario
     */
    fun startDrivingScenario(scenarioId: String): Boolean {
        if (!_twoWayCommEnabled.value || scenarioId.isBlank()) return false
        return obdService.controlSimulation("start", scenarioId)
    }
    
    /**
     * Stop the current simulation/scenario
     */
    fun stopSimulation(): Boolean {
        if (!_twoWayCommEnabled.value) return false
        return obdService.controlSimulation("stop")
    }
    
    /**
     * Adjust an emulator setting
     */
    fun adjustEmulatorSetting(setting: OBDService.EmulatorSetting, value: Float): Boolean {
        if (!_twoWayCommEnabled.value) return false
        
        val currentSettings = _emulatorSettings.value.toMutableMap()
        currentSettings[setting] = value
        _emulatorSettings.value = currentSettings
        
        return obdService.adjustSetting(setting, value)
    }
    
    /**
     * Start a simulation profile based on a stored DrivingProfile
     */
    fun startProfileSimulation(profile: DrivingProfile): Boolean {
        if (!_twoWayCommEnabled.value) return false
        
        // Extract first curve if available
        val firstCurve = profile.getCurve(0) ?: return false
        
        // Apply profile settings first
        profile.defaultSettings.forEach { (key, value) ->
            when (key) {
                "animation_speed" -> updateAnimationSpeed(value)
                "engine_temp_factor" -> adjustEmulatorSetting(OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR, value)
                "fuel_consumption_factor" -> adjustEmulatorSetting(OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR, value)
                "rpm_response_factor" -> adjustEmulatorSetting(OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR, value)
                else -> { /* Ignore unknown settings */ }
            }
        }
        
        // Convert the curve to parameters
        val parameters = mapOf(
            "name" to firstCurve.name,
            "duration" to firstCurve.duration().toString(),
            "points" to firstCurve.points.joinToString(",") { "${it.x}:${it.y}" }
        )
        
        // Trigger the profile with the curve data
        return obdService.triggerSimulationProfile(profile.id, parameters)
    }

    /**
     * Dismiss a warning notification by its ID
     */
    fun dismissWarning(warningId: String) {
        viewModelScope.launch(Dispatchers.Main) {
            _activeWarnings.removeIf { it.id == warningId }
            dismissedWarningIds.add(warningId)
        }
    }
    
    /**
     * Dismiss all warning notifications
     */
    fun dismissAllWarnings() {
        viewModelScope.launch(Dispatchers.Main) {
            _activeWarnings.forEach { warning ->
                dismissedWarningIds.add(warning.id)
            }
            _activeWarnings.clear()
        }
    }
    
    /**
     * Update all vehicle systems status based on current vehicle parameters
     */
    private fun updateVehicleSystemStatus() {
        // Engine status
        when {
            engineTemperature > 110 -> {
                engineSystemStatus = SystemStatus.CRITICAL
                engineSystemDetails = "Engine temperature critically high (${engineTemperature.toInt()}°C)"
                addWarningIfNotExists(
                    "engine_temp",
                    "Engine Overheating",
                    "Engine temperature critically high at ${engineTemperature.toInt()}°C. Risk of engine damage.",
                    WarningLevel.CRITICAL,
                    true,
                    "Pull over safely and turn off the engine"
                )
            }
            engineTemperature > 100 -> {
                engineSystemStatus = SystemStatus.WARNING
                engineSystemDetails = "Engine temperature high (${engineTemperature.toInt()}°C)"
                addWarningIfNotExists(
                    "engine_temp",
                    "Engine Temperature High",
                    "Engine temperature elevated at ${engineTemperature.toInt()}°C.",
                    WarningLevel.WARNING
                )
            }
            engineTemperature < 60 && speed > 0 -> {
                engineSystemStatus = SystemStatus.WARNING
                engineSystemDetails = "Engine temperature low (${engineTemperature.toInt()}°C)"
                addWarningIfNotExists(
                    "engine_temp_low",
                    "Engine Temperature Low",
                    "Engine not at optimal temperature (${engineTemperature.toInt()}°C).",
                    WarningLevel.WARNING
                )
            }
            else -> {
                engineSystemStatus = SystemStatus.NORMAL
                engineSystemDetails = ""
                removeWarning("engine_temp")
                removeWarning("engine_temp_low")
            }
        }
        
        // Oil pressure status
        when {
            oilPressure < 10 -> {
                engineSystemStatus = SystemStatus.CRITICAL
                engineSystemDetails = "Oil pressure critically low ($oilPressure PSI)"
                addWarningIfNotExists(
                    "oil_pressure",
                    "Low Oil Pressure",
                    "Oil pressure critically low at $oilPressure PSI. Risk of engine damage.",
                    WarningLevel.CRITICAL,
                    true,
                    "Pull over safely and check oil level"
                )
            }
            oilPressure < 20 -> {
                engineSystemStatus = SystemStatus.WARNING
                engineSystemDetails = "Oil pressure low ($oilPressure PSI)"
                addWarningIfNotExists(
                    "oil_pressure",
                    "Low Oil Pressure",
                    "Oil pressure below recommended level at $oilPressure PSI. Check oil level.",
                    WarningLevel.WARNING
                )
            }
            else -> {
                // Only update if not already in critical/warning state
                if (engineSystemStatus == SystemStatus.NORMAL) {
                    engineSystemDetails = ""
                }
                removeWarning("oil_pressure")
            }
        }
        
        // Fuel system status
        when {
            fuelLevel < 10 -> {
                fuelSystemStatus = SystemStatus.WARNING
                fuelSystemDetails = "Fuel level low (${fuelLevel.toInt()}%)"
                addWarningIfNotExists(
                    "fuel_level",
                    "Low Fuel Level",
                    "Fuel level at ${fuelLevel.toInt()}%. Estimated range: ${estimatedRange.toInt()}km.",
                    WarningLevel.WARNING,
                    true,
                    "Refuel soon"
                )
            }
            else -> {
                fuelSystemStatus = SystemStatus.NORMAL
                fuelSystemDetails = ""
                removeWarning("fuel_level")
            }
        }
        
        // Battery system status
        when {
            batteryVoltage < 11.8 -> {
                batterySystemStatus = SystemStatus.CRITICAL
                batterySystemDetails = "Battery voltage critically low (${batteryVoltage}V)"
                addWarningIfNotExists(
                    "battery_voltage",
                    "Low Battery Voltage",
                    "Battery voltage critically low at ${batteryVoltage}V. Charging system may be faulty.",
                    WarningLevel.CRITICAL,
                    true,
                    "Check battery and charging system"
                )
            }
            batteryVoltage < 12.2 -> {
                batterySystemStatus = SystemStatus.WARNING
                batterySystemDetails = "Battery voltage low (${batteryVoltage}V)"
                addWarningIfNotExists(
                    "battery_voltage",
                    "Low Battery Voltage",
                    "Battery voltage low at ${batteryVoltage}V.",
                    WarningLevel.WARNING
                )
            }
            batteryVoltage > 14.8 -> {
                batterySystemStatus = SystemStatus.WARNING
                batterySystemDetails = "Battery voltage high (${batteryVoltage}V)"
                addWarningIfNotExists(
                    "battery_voltage_high",
                    "High Battery Voltage",
                    "Battery voltage high at ${batteryVoltage}V. Charging system may be faulty.",
                    WarningLevel.WARNING
                )
            }
            else -> {
                batterySystemStatus = SystemStatus.NORMAL
                batterySystemDetails = ""
                removeWarning("battery_voltage")
                removeWarning("battery_voltage_high")
            }
        }
        
        // Update overall driving scores
        updateDrivingScores()
    }
    
    /**
     * Update all driving style scores based on current driving parameters
     */
    private fun updateDrivingScores() {
        // Update acceleration score based on acceleration value and throttle position
        accelerationScore = when {
            acceleration > 3.0f -> 0.0f
            acceleration > 2.5f -> 0.3f
            acceleration > 2.0f -> 0.5f
            acceleration > 1.5f -> 0.7f
            else -> 1.0f
        }
        
        // Update deceleration score based on negative acceleration (braking)
        decelerationScore = when {
            acceleration < -3.0f -> 0.0f
            acceleration < -2.5f -> 0.3f
            acceleration < -2.0f -> 0.5f
            acceleration < -1.5f -> 0.7f
            else -> 1.0f
        }
        
        // Update engine efficiency score based on RPM and speed ratio
        val rpmPerSpeed = if (speed > 10f) rpm / speed else 0f
        engineEfficiencyScore = when {
            rpmPerSpeed > 100f -> 0.3f
            rpmPerSpeed > 80f -> 0.5f
            rpmPerSpeed > 60f -> 0.7f
            rpmPerSpeed > 40f -> 0.9f
            else -> 1.0f
        }
        
        // Update cornering score (simple simulation)
        corneringScore = if (Random.nextFloat() < 0.1f) {
            (corneringScore * 0.9f + Random.nextFloat() * 0.1f).coerceIn(0.2f, 1.0f)
        } else {
            (corneringScore * 0.95f + 0.05f).coerceIn(0.5f, 1.0f)
        }
        
        // Update overall metrics
        efficiencyScore = (engineEfficiencyScore * 0.7f + 
                          (1.0f - throttlePosition) * 0.3f).coerceIn(0.0f, 1.0f)
        
        smoothnessScore = (accelerationScore * 0.4f + 
                          decelerationScore * 0.4f + 
                          corneringScore * 0.2f).coerceIn(0.0f, 1.0f)
        
        overallDrivingScore = (efficiencyScore * 40f + 
                              smoothnessScore * 30f + 
                              comfortLevel * 30f).coerceIn(0f, 100f)
    }
    
    /**
     * Add a warning notification if one with the same ID doesn't already exist and hasn't been dismissed
     */
    private fun addWarningIfNotExists(
        id: String,
        title: String,
        description: String,
        level: WarningLevel,
        actionRequired: Boolean = false,
        recommendedAction: String = ""
    ) {
        viewModelScope.launch(Dispatchers.Main) {
            if (_activeWarnings.none { it.id == id } && !dismissedWarningIds.contains(id)) {
                _activeWarnings.add(
                    VehicleWarning(
                        id = id,
                        title = title,
                        description = description,
                        level = level,
                        timestamp = LocalDateTime.now(),
                        isNew = true,
                        actionRequired = actionRequired,
                        recommendedAction = recommendedAction
                    )
                )
            }
        }
    }
    
    /**
     * Remove a warning notification by its ID
     */
    private fun removeWarning(id: String) {
        viewModelScope.launch(Dispatchers.Main) {
            _activeWarnings.removeIf { it.id == id }
            // Don't add to dismissedWarningIds here as this is a system-initiated removal
        }
    }

    /**
     * Simulate OBD data for development and testing
     */
    private fun simulateOBDData() {
        viewModelScope.launch(Dispatchers.Main) {
            while (true) {
                // Simulate acceleration/deceleration
                val targetSpeed = when {
                    Random.nextFloat() < 0.05f -> 0f // Occasionally stop
                    Random.nextFloat() < 0.1f -> Random.nextFloat() * 120f // Change speed target
                    else -> speed
                }
                
                // Gradual approach to target speed
                speed = if (targetSpeed > speed) {
                    acceleration = (0.2f + Random.nextFloat() * 0.8f).coerceIn(0.1f, 3.0f)
                    (speed + acceleration).coerceIn(0f, 200f)
                } else if (targetSpeed < speed) {
                    acceleration = (-0.2f - Random.nextFloat() * 1.8f).coerceIn(-3.0f, -0.1f)
                    (speed + acceleration).coerceIn(0f, 200f)
                } else {
                    acceleration = 0f
                    speed
                }
                
                // Update engine RPM based on speed
                val targetRPM = if (speed < 5f) 800f else 800f + (speed * 25f) + Random.nextFloat() * 500f
                rpm = ((rpm * 9f + targetRPM) / 10f).coerceIn(800f, 6000f)
                
                // Simulate gear changes
                updateGearPosition(speed)
                
                // Update throttle and brake positions
                throttlePosition = if (acceleration > 0f) {
                    (acceleration / 3f).coerceIn(0f, 1f)
                } else {
                    0f
                }
                
                brakePosition = if (acceleration < 0f) {
                    (-acceleration / 3f).coerceIn(0f, 1f)
                } else {
                    0f
                }
                
                // Update engine temperature (slowly increases with RPM, slowly decreases over time)
                val tempChange = ((rpm - 800f) / 10000f) - 0.1f
                engineTemperature = (engineTemperature + tempChange).coerceIn(50f, 130f)
                
                // Add potential random spikes or drops in temperature occasionally
                if (Random.nextFloat() < 0.01f) {
                    engineTemperature += Random.nextFloat() * 10f - 5f
                }
                
                // Update fuel level (decreases slowly with usage)
                val fuelUsage = 0.01f * (throttlePosition * 2f + 0.1f)
                fuelLevel = (fuelLevel - fuelUsage).coerceIn(0f, 100f)
                
                // Update fuel consumption rate based on throttle and speed
                fuelConsumptionRate = if (speed > 5f) {
                    (5f + throttlePosition * 10f + Random.nextFloat()).coerceIn(5f, 15f)
                } else {
                    0.5f
                }
                
                // Update average consumption (slowly trends toward current consumption)
                averageFuelConsumption = ((averageFuelConsumption * 99f + fuelConsumptionRate) / 100f).coerceIn(5f, 12f)
                
                // Update estimated range
                estimatedRange = (fuelLevel / 100f) * 50f * (600f / averageFuelConsumption)
                
                // Update battery voltage (random fluctuations)
                batteryVoltage = (batteryVoltage * 0.98f + (Random.nextFloat() * 0.4f + 12.4f) * 0.02f).coerceIn(11.5f, 15.0f)
                
                // Add potential battery issues occasionally
                if (Random.nextFloat() < 0.01f) {
                    batteryVoltage += Random.nextFloat() * 0.6f - 0.3f
                }
                
                // Update oil pressure (correlates with RPM)
                oilPressure = (rpm / 200f + Random.nextFloat() * 5f).coerceIn(5f, 60f)
                
                // Occasionally simulate oil pressure issues
                if (Random.nextFloat() < 0.01f) {
                    oilPressure -= Random.nextFloat() * 15f
                }
                
                // Update driving state based on acceleration
                drivingState = when {
                    speed < 5f -> DrivingState.IDLE
                    acceleration > 2.0f -> DrivingState.AGGRESSIVE_ACCELERATION
                    acceleration < -2.0f -> DrivingState.AGGRESSIVE_DECELERATION
                    else -> DrivingState.NORMAL
                }
                
                // Update comfort level based on acceleration changes
                comfortLevel = (1.0f - (acceleration.coerceIn(-3f, 3f).absoluteValue / 3f)).coerceIn(0.1f, 1.0f)
                
                // Update aggressive score list
                val aggressiveScore = when {
                    drivingState == DrivingState.AGGRESSIVE_ACCELERATION ||
                    drivingState == DrivingState.AGGRESSIVE_DECELERATION -> 1.0f
                    else -> 0.0f
                }
                
                aggressiveScores.add(aggressiveScore)
                if (aggressiveScores.size > 20) {
                    aggressiveScores.removeAt(0)
                }
                
                // Update temperature history
                val now = System.currentTimeMillis()
                temperatureHistory.add(Pair(now, engineTemperature))
                if (temperatureHistory.size > 100) {
                    temperatureHistory.removeAt(0)
                }
                
                // Update vehicle status
                updateVehicleSystemStatus()
                
                delay(500)
            }
        }
    }

    private fun calculateAcceleration(currentSpeed: Float): Float {
        val previousSpeed = speedHistory.lastOrNull()?.second ?: currentSpeed
        val timeDiff = 0.1f // 100ms update interval
        return (currentSpeed - previousSpeed) / timeDiff
    }
}