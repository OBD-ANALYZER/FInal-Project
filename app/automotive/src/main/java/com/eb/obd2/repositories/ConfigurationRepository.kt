package com.eb.obd2.repositories

import android.content.Context
import android.util.Log
import com.eb.obd2.models.ConnectionConfig
import com.eb.obd2.models.SimulationProfile
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import java.util.UUID
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Repository for managing connection configurations and simulation profiles
 */
@Singleton
class ConfigurationRepository @Inject constructor(
    private val context: Context
) {
    private val TAG = "ConfigRepository"
    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }
    
    // Directory for configuration files
    private val configDir by lazy {
        File(context.filesDir, "config").apply {
            if (!exists()) mkdirs()
        }
    }
    
    // Connection configurations file
    private val connectionsFile by lazy {
        File(configDir, "connections.json")
    }
    
    // Simulation profiles file
    private val profilesFile by lazy {
        File(configDir, "profiles.json")
    }
    
    // Connection configurations
    private val _connectionConfigs = MutableStateFlow<List<ConnectionConfig>>(emptyList())
    val connectionConfigs: StateFlow<List<ConnectionConfig>> = _connectionConfigs.asStateFlow()
    
    // Simulation profiles
    private val _simulationProfiles = MutableStateFlow<List<SimulationProfile>>(emptyList())
    val simulationProfiles: StateFlow<List<SimulationProfile>> = _simulationProfiles.asStateFlow()
    
    // Current active configuration
    private val _activeConfig = MutableStateFlow<ConnectionConfig?>(null)
    val activeConfig: StateFlow<ConnectionConfig?> = _activeConfig.asStateFlow()
    
    // Current active profile
    private val _activeProfile = MutableStateFlow<SimulationProfile?>(null)
    val activeProfile: StateFlow<SimulationProfile?> = _activeProfile.asStateFlow()
    
    init {
        // Load saved configurations and profiles
        CoroutineScope(Dispatchers.IO).launch {
            loadConnectionConfigs()
            loadSimulationProfiles()
            
            // Set active config and profile to defaults if available
            _connectionConfigs.value.find { it.isDefault }?.let {
                _activeConfig.value = it
            }
            
            _simulationProfiles.value.find { it.isDefault }?.let {
                _activeProfile.value = it
            }
            
            // If no configs exist, create defaults
            if (_connectionConfigs.value.isEmpty()) {
                val defaultConfig = ConnectionConfig.createDefault()
                saveConnectionConfig(defaultConfig)
                _activeConfig.value = defaultConfig
                
                // Create a default profile using this connection
                val defaultProfile = SimulationProfile.createDefault(defaultConfig.id)
                saveSimulationProfile(defaultProfile)
                _activeProfile.value = defaultProfile
            }
        }
    }
    
    /**
     * Load connection configurations from disk
     */
    private fun loadConnectionConfigs() {
        try {
            if (connectionsFile.exists()) {
                val configsJson = connectionsFile.readText()
                val configs = json.decodeFromString<List<ConnectionConfig>>(configsJson)
                _connectionConfigs.value = configs
                Log.d(TAG, "Loaded ${configs.size} connection configurations")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load connection configurations", e)
            // If loading fails, start with an empty list
            _connectionConfigs.value = emptyList()
        }
    }
    
    /**
     * Load simulation profiles from disk
     */
    private fun loadSimulationProfiles() {
        try {
            if (profilesFile.exists()) {
                val profilesJson = profilesFile.readText()
                val profiles = json.decodeFromString<List<SimulationProfile>>(profilesJson)
                _simulationProfiles.value = profiles
                Log.d(TAG, "Loaded ${profiles.size} simulation profiles")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load simulation profiles", e)
            // If loading fails, start with an empty list
            _simulationProfiles.value = emptyList()
        }
    }
    
    /**
     * Save connection configurations to disk
     */
    private fun saveConnectionConfigs() {
        try {
            val configsJson = json.encodeToString(_connectionConfigs.value)
            connectionsFile.writeText(configsJson)
            Log.d(TAG, "Saved ${_connectionConfigs.value.size} connection configurations")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save connection configurations", e)
        }
    }
    
    /**
     * Save simulation profiles to disk
     */
    private fun saveSimulationProfiles() {
        try {
            val profilesJson = json.encodeToString(_simulationProfiles.value)
            profilesFile.writeText(profilesJson)
            Log.d(TAG, "Saved ${_simulationProfiles.value.size} simulation profiles")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save simulation profiles", e)
        }
    }
    
    /**
     * Save a connection configuration
     * If it's a new configuration (id not found), it will be added
     * If it already exists, it will be updated
     * 
     * @param config The connection configuration to save
     * @return The saved configuration
     */
    fun saveConnectionConfig(config: ConnectionConfig): ConnectionConfig {
        val currentConfigs = _connectionConfigs.value.toMutableList()
        
        // Check if this is set as default, and clear other defaults if needed
        val updatedConfig = if (config.isDefault) {
            // Clear default flag from all other configs
            val updatedConfigs = currentConfigs.map { 
                if (it.id != config.id && it.isDefault) it.withDefaultStatus(false) else it 
            }.toMutableList()
            
            // Update the configs list
            currentConfigs.clear()
            currentConfigs.addAll(updatedConfigs)
            
            config
        } else {
            config
        }
        
        // Find existing config with the same ID
        val existingIndex = currentConfigs.indexOfFirst { it.id == config.id }
        
        if (existingIndex >= 0) {
            // Update existing config
            currentConfigs[existingIndex] = updatedConfig
        } else {
            // Add new config
            currentConfigs.add(updatedConfig)
        }
        
        _connectionConfigs.value = currentConfigs
        saveConnectionConfigs()
        
        // If this is the only config, make it active
        if (currentConfigs.size == 1 || updatedConfig.isDefault) {
            _activeConfig.value = updatedConfig
        }
        
        return updatedConfig
    }
    
    /**
     * Create a new connection configuration
     * 
     * @param name Name for the configuration
     * @param host Connection host/IP
     * @param port Connection port
     * @param useBluetooth Whether to use Bluetooth
     * @param bluetoothAddress Bluetooth device address (required if useBluetooth is true)
     * @param autoReconnect Whether to auto-reconnect
     * @return The newly created configuration
     */
    fun createConnectionConfig(
        name: String,
        host: String,
        port: Int,
        useBluetooth: Boolean = false,
        bluetoothAddress: String? = null,
        autoReconnect: Boolean = true
    ): ConnectionConfig {
        val id = UUID.randomUUID().toString()
        val config = ConnectionConfig(
            id = id,
            name = name,
            host = host,
            port = port,
            useBluetooth = useBluetooth,
            bluetoothAddress = bluetoothAddress,
            autoReconnect = autoReconnect
        )
        
        return saveConnectionConfig(config)
    }
    
    /**
     * Delete a connection configuration
     * 
     * @param configId ID of the configuration to delete
     * @return true if successful, false otherwise
     */
    fun deleteConnectionConfig(configId: String): Boolean {
        // Cannot delete the active config
        if (_activeConfig.value?.id == configId) {
            return false
        }
        
        val currentConfigs = _connectionConfigs.value.toMutableList()
        val removed = currentConfigs.removeIf { it.id == configId }
        
        if (removed) {
            _connectionConfigs.value = currentConfigs
            saveConnectionConfigs()
            
            // Delete any profiles that use this connection
            val affectedProfiles = _simulationProfiles.value.filter { it.connectionConfigId == configId }
            affectedProfiles.forEach { deleteSimulationProfile(it.id) }
        }
        
        return removed
    }
    
    /**
     * Get a connection configuration by ID
     * 
     * @param configId ID of the configuration to get
     * @return The configuration, or null if not found
     */
    fun getConnectionConfig(configId: String): ConnectionConfig? {
        return _connectionConfigs.value.find { it.id == configId }
    }
    
    /**
     * Set the active connection configuration
     * 
     * @param configId ID of the configuration to set as active
     * @return The activated configuration, or null if not found
     */
    fun setActiveConfig(configId: String): ConnectionConfig? {
        val config = getConnectionConfig(configId) ?: return null
        _activeConfig.value = config
        return config
    }
    
    /**
     * Save a simulation profile
     * If it's a new profile (id not found), it will be added
     * If it already exists, it will be updated
     * 
     * @param profile The simulation profile to save
     * @return The saved profile
     */
    fun saveSimulationProfile(profile: SimulationProfile): SimulationProfile {
        val currentProfiles = _simulationProfiles.value.toMutableList()
        
        // Check if this is set as default, and clear other defaults if needed
        val updatedProfile = if (profile.isDefault) {
            // Clear default flag from all other profiles
            val updatedProfiles = currentProfiles.map { 
                if (it.id != profile.id && it.isDefault) it.withDefaultStatus(false) else it 
            }.toMutableList()
            
            // Update the profiles list
            currentProfiles.clear()
            currentProfiles.addAll(updatedProfiles)
            
            profile
        } else {
            profile
        }
        
        // Find existing profile with the same ID
        val existingIndex = currentProfiles.indexOfFirst { it.id == profile.id }
        
        if (existingIndex >= 0) {
            // Update existing profile
            currentProfiles[existingIndex] = updatedProfile
        } else {
            // Add new profile
            currentProfiles.add(updatedProfile)
        }
        
        _simulationProfiles.value = currentProfiles
        saveSimulationProfiles()
        
        // If this is the only profile, make it active
        if (currentProfiles.size == 1 || updatedProfile.isDefault) {
            _activeProfile.value = updatedProfile
        }
        
        return updatedProfile
    }
    
    /**
     * Create a new simulation profile
     * 
     * @param name Name for the profile
     * @param description Description of the profile
     * @param connectionConfigId ID of the connection configuration to use
     * @param emulatorSettings Map of emulator settings
     * @param initialParameters Map of initial vehicle parameters
     * @return The newly created profile
     */
    fun createSimulationProfile(
        name: String,
        description: String,
        connectionConfigId: String,
        emulatorSettings: Map<String, Float> = emptyMap(),
        initialParameters: Map<String, Float> = emptyMap()
    ): SimulationProfile {
        val id = UUID.randomUUID().toString()
        val profile = SimulationProfile(
            id = id,
            name = name,
            description = description,
            connectionConfigId = connectionConfigId,
            emulatorSettings = emulatorSettings,
            initialParameters = initialParameters
        )
        
        return saveSimulationProfile(profile)
    }
    
    /**
     * Delete a simulation profile
     * 
     * @param profileId ID of the profile to delete
     * @return true if successful, false otherwise
     */
    fun deleteSimulationProfile(profileId: String): Boolean {
        // Cannot delete the active profile
        if (_activeProfile.value?.id == profileId) {
            return false
        }
        
        val currentProfiles = _simulationProfiles.value.toMutableList()
        val removed = currentProfiles.removeIf { it.id == profileId }
        
        if (removed) {
            _simulationProfiles.value = currentProfiles
            saveSimulationProfiles()
        }
        
        return removed
    }
    
    /**
     * Get a simulation profile by ID
     * 
     * @param profileId ID of the profile to get
     * @return The profile, or null if not found
     */
    fun getSimulationProfile(profileId: String): SimulationProfile? {
        return _simulationProfiles.value.find { it.id == profileId }
    }
    
    /**
     * Set the active simulation profile
     * 
     * @param profileId ID of the profile to set as active
     * @return The activated profile, or null if not found
     */
    fun setActiveProfile(profileId: String): SimulationProfile? {
        val profile = getSimulationProfile(profileId) ?: return null
        _activeProfile.value = profile
        return profile
    }
    
    /**
     * Get preset simulation profiles
     * 
     * @return List of preset simulation profiles
     */
    fun getPresetProfiles(): List<SimulationProfile> {
        // Get default connection config ID
        val defaultConnectionId = _connectionConfigs.value.find { it.isDefault }?.id
            ?: _connectionConfigs.value.firstOrNull()?.id
            ?: return emptyList()
        
        return listOf(
            SimulationProfile(
                id = "preset_urban",
                name = "Urban Driving",
                description = "Typical city driving with frequent stops and moderate speeds",
                connectionConfigId = defaultConnectionId,
                emulatorSettings = mapOf(
                    "engine_temp_factor" to 1.2f,
                    "fuel_consumption_factor" to 1.3f,
                    "rpm_response_factor" to 1.0f,
                    "simulation_speed" to 1.0f
                ),
                initialParameters = mapOf(
                    "speed" to 35f,
                    "rpm" to 1500f,
                    "engine_temp" to 80f,
                    "fuel_level" to 75f
                )
            ),
            SimulationProfile(
                id = "preset_highway",
                name = "Highway Cruising",
                description = "Highway driving at sustained high speeds",
                connectionConfigId = defaultConnectionId,
                emulatorSettings = mapOf(
                    "engine_temp_factor" to 1.1f,
                    "fuel_consumption_factor" to 0.8f,
                    "rpm_response_factor" to 0.9f,
                    "simulation_speed" to 1.5f
                ),
                initialParameters = mapOf(
                    "speed" to 120f,
                    "rpm" to 2500f,
                    "engine_temp" to 90f,
                    "fuel_level" to 80f
                )
            ),
            SimulationProfile(
                id = "preset_cold_start",
                name = "Cold Start",
                description = "Vehicle start in cold weather conditions",
                connectionConfigId = defaultConnectionId,
                emulatorSettings = mapOf(
                    "engine_temp_factor" to 0.7f,
                    "fuel_consumption_factor" to 1.5f,
                    "rpm_response_factor" to 1.2f,
                    "simulation_speed" to 0.8f
                ),
                initialParameters = mapOf(
                    "speed" to 0f,
                    "rpm" to 800f,
                    "engine_temp" to 20f,
                    "fuel_level" to 45f
                )
            ),
            SimulationProfile(
                id = "preset_mountain",
                name = "Mountain Driving",
                description = "Driving on mountain roads with steep gradients",
                connectionConfigId = defaultConnectionId,
                emulatorSettings = mapOf(
                    "engine_temp_factor" to 1.4f,
                    "fuel_consumption_factor" to 1.6f,
                    "rpm_response_factor" to 1.3f,
                    "simulation_speed" to 0.9f
                ),
                initialParameters = mapOf(
                    "speed" to 40f,
                    "rpm" to 3000f,
                    "engine_temp" to 95f,
                    "fuel_level" to 60f
                )
            )
        )
    }
    
    /**
     * Import a preset profile
     * 
     * @param presetId ID of the preset profile to import
     * @return The imported profile, or null if preset not found
     */
    fun importPresetProfile(presetId: String): SimulationProfile? {
        val preset = getPresetProfiles().find { it.id == presetId } ?: return null
        
        // Create a new ID for the imported profile
        val importedProfile = preset.copy(
            id = UUID.randomUUID().toString(),
            name = "${preset.name} (Imported)"
        )
        
        return saveSimulationProfile(importedProfile)
    }
} 