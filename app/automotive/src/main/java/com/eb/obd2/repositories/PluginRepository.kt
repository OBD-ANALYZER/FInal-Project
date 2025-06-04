package com.eb.obd2.repositories

import android.content.Context
import android.util.Log
import com.eb.obd2.models.Plugin
import com.eb.obd2.models.PluginState
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import java.net.URL
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Repository for managing OBD-II emulator plugins
 */
@Singleton
class PluginRepository @Inject constructor(
    private val context: Context
) {
    private val TAG = "PluginRepository"
    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }
    
    // Directory for plugin files
    private val pluginsDir by lazy {
        File(context.filesDir, "plugins").apply {
            if (!exists()) mkdirs()
        }
    }
    
    // Directory for downloaded plugin binaries
    private val pluginBinariesDir by lazy {
        File(pluginsDir, "binaries").apply {
            if (!exists()) mkdirs()
        }
    }
    
    // Plugins metadata file
    private val pluginsFile by lazy {
        File(pluginsDir, "plugins.json")
    }
    
    // Plugin states file
    private val pluginStatesFile by lazy {
        File(pluginsDir, "plugin_states.json")
    }
    
    // Available plugins
    private val _availablePlugins = MutableStateFlow<List<Plugin>>(emptyList())
    val availablePlugins: StateFlow<List<Plugin>> = _availablePlugins.asStateFlow()
    
    // Installed plugins
    private val _installedPlugins = MutableStateFlow<List<Plugin>>(emptyList())
    val installedPlugins: StateFlow<List<Plugin>> = _installedPlugins.asStateFlow()
    
    // Plugin states
    private val _pluginStates = MutableStateFlow<Map<String, PluginState>>(emptyMap())
    val pluginStates: StateFlow<Map<String, PluginState>> = _pluginStates.asStateFlow()
    
    // Default plugin server URL
    private val defaultPluginServerUrl = "https://obd2-plugins.example.com/api/plugins"
    
    // Current plugin server URL
    private var pluginServerUrl = defaultPluginServerUrl
    
    init {
        // Load saved plugins and states
        CoroutineScope(Dispatchers.IO).launch {
            loadInstalledPlugins()
            loadPluginStates()
        }
    }
    
    /**
     * Load installed plugins from disk
     */
    private fun loadInstalledPlugins() {
        try {
            if (pluginsFile.exists()) {
                val pluginsJson = pluginsFile.readText()
                val plugins = json.decodeFromString<List<Plugin>>(pluginsJson)
                _installedPlugins.value = plugins
                Log.d(TAG, "Loaded ${plugins.size} installed plugins")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load installed plugins", e)
            // If loading fails, start with an empty list
            _installedPlugins.value = emptyList()
        }
    }
    
    /**
     * Load plugin states from disk
     */
    private fun loadPluginStates() {
        try {
            if (pluginStatesFile.exists()) {
                val statesJson = pluginStatesFile.readText()
                val states = json.decodeFromString<List<PluginState>>(statesJson)
                _pluginStates.value = states.associateBy { it.pluginId }
                Log.d(TAG, "Loaded ${states.size} plugin states")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load plugin states", e)
            // If loading fails, start with an empty map
            _pluginStates.value = emptyMap()
        }
    }
    
    /**
     * Save installed plugins to disk
     */
    private fun saveInstalledPlugins() {
        try {
            val pluginsJson = json.encodeToString(_installedPlugins.value)
            pluginsFile.writeText(pluginsJson)
            Log.d(TAG, "Saved ${_installedPlugins.value.size} installed plugins")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save installed plugins", e)
        }
    }
    
    /**
     * Save plugin states to disk
     */
    private fun savePluginStates() {
        try {
            val statesJson = json.encodeToString(_pluginStates.value.values.toList())
            pluginStatesFile.writeText(statesJson)
            Log.d(TAG, "Saved ${_pluginStates.value.size} plugin states")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save plugin states", e)
        }
    }
    
    /**
     * Set the plugin server URL
     * 
     * @param url The server URL
     */
    fun setPluginServerUrl(url: String) {
        pluginServerUrl = url
    }
    
    /**
     * Get the current plugin server URL
     * 
     * @return The current server URL
     */
    fun getPluginServerUrl(): String {
        return pluginServerUrl
    }
    
    /**
     * Reset the plugin server URL to the default
     */
    fun resetPluginServerUrl() {
        pluginServerUrl = defaultPluginServerUrl
    }
    
    /**
     * Fetch available plugins from the server
     * 
     * @return True if successful, false otherwise
     */
    suspend fun fetchAvailablePlugins(): Boolean {
        return withContext(Dispatchers.IO) {
            try {
                val url = URL(pluginServerUrl)
                val connection = url.openConnection()
                connection.connectTimeout = 5000
                connection.readTimeout = 5000
                
                val inputStream = connection.getInputStream()
                val responseJson = inputStream.bufferedReader().use { it.readText() }
                
                val plugins = json.decodeFromString<List<Plugin>>(responseJson)
                _availablePlugins.value = plugins
                
                Log.d(TAG, "Fetched ${plugins.size} available plugins")
                true
            } catch (e: Exception) {
                Log.e(TAG, "Failed to fetch available plugins", e)
                false
            }
        }
    }
    
    /**
     * Install a plugin from the available plugins
     * 
     * @param pluginId ID of the plugin to install
     * @return True if successful, false otherwise
     */
    suspend fun installPlugin(pluginId: String): Boolean {
        return withContext(Dispatchers.IO) {
            try {
                // Find the plugin in available plugins
                val plugin = _availablePlugins.value.find { it.id == pluginId }
                    ?: return@withContext false
                
                // Download the plugin binary
                val url = URL(plugin.downloadUrl)
                val connection = url.openConnection()
                connection.connectTimeout = 5000
                connection.readTimeout = 30000
                
                val pluginFileName = "${plugin.id}_${plugin.version}.jar"
                val pluginFile = File(pluginBinariesDir, pluginFileName)
                
                val inputStream = connection.getInputStream()
                pluginFile.outputStream().use { output ->
                    inputStream.copyTo(output)
                }
                
                // Create installed plugin with local path
                val installedPlugin = plugin.copy(
                    localPath = pluginFile.absolutePath,
                    enabled = false
                )
                
                // Add to installed plugins
                val currentPlugins = _installedPlugins.value.toMutableList()
                currentPlugins.add(installedPlugin)
                _installedPlugins.value = currentPlugins
                
                // Create initial plugin state
                val pluginState = PluginState(pluginId = plugin.id)
                val currentStates = _pluginStates.value.toMutableMap()
                currentStates[plugin.id] = pluginState
                _pluginStates.value = currentStates
                
                // Save changes
                saveInstalledPlugins()
                savePluginStates()
                
                Log.d(TAG, "Installed plugin: ${plugin.name}")
                true
            } catch (e: Exception) {
                Log.e(TAG, "Failed to install plugin", e)
                false
            }
        }
    }
    
    /**
     * Uninstall a plugin
     * 
     * @param pluginId ID of the plugin to uninstall
     * @return True if successful, false otherwise
     */
    fun uninstallPlugin(pluginId: String): Boolean {
        try {
            // Find the plugin
            val plugin = _installedPlugins.value.find { it.id == pluginId }
                ?: return false
            
            // Delete the plugin binary if it exists
            plugin.localPath?.let {
                val pluginFile = File(it)
                if (pluginFile.exists()) {
                    pluginFile.delete()
                }
            }
            
            // Remove from installed plugins
            val currentPlugins = _installedPlugins.value.toMutableList()
            currentPlugins.removeIf { it.id == pluginId }
            _installedPlugins.value = currentPlugins
            
            // Remove plugin state
            val currentStates = _pluginStates.value.toMutableMap()
            currentStates.remove(pluginId)
            _pluginStates.value = currentStates
            
            // Save changes
            saveInstalledPlugins()
            savePluginStates()
            
            Log.d(TAG, "Uninstalled plugin: ${plugin.name}")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to uninstall plugin", e)
            return false
        }
    }
    
    /**
     * Enable or disable a plugin
     * 
     * @param pluginId ID of the plugin
     * @param enabled Whether to enable or disable the plugin
     * @return True if successful, false otherwise
     */
    fun setPluginEnabled(pluginId: String, enabled: Boolean): Boolean {
        try {
            // Find the plugin
            val currentPlugins = _installedPlugins.value.toMutableList()
            val index = currentPlugins.indexOfFirst { it.id == pluginId }
            
            if (index < 0) {
                return false
            }
            
            // Update plugin enabled state
            val plugin = currentPlugins[index]
            currentPlugins[index] = plugin.copy(enabled = enabled)
            _installedPlugins.value = currentPlugins
            
            // Update plugin active state
            val currentStates = _pluginStates.value.toMutableMap()
            val state = currentStates[pluginId] ?: PluginState(pluginId = pluginId)
            currentStates[pluginId] = state.copy(active = enabled)
            _pluginStates.value = currentStates
            
            // Save changes
            saveInstalledPlugins()
            savePluginStates()
            
            Log.d(TAG, "Set plugin ${plugin.name} enabled: $enabled")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to set plugin enabled state", e)
            return false
        }
    }
    
    /**
     * Update plugin configuration option
     * 
     * @param pluginId ID of the plugin
     * @param optionId ID of the configuration option
     * @param value New value for the option
     * @return True if successful, false otherwise
     */
    fun updatePluginConfig(pluginId: String, optionId: String, value: String): Boolean {
        try {
            // Find the plugin
            val currentPlugins = _installedPlugins.value.toMutableList()
            val index = currentPlugins.indexOfFirst { it.id == pluginId }
            
            if (index < 0) {
                return false
            }
            
            // Update configuration option
            val plugin = currentPlugins[index]
            val configOptions = plugin.configOptions.toMutableList()
            val optionIndex = configOptions.indexOfFirst { it.id == optionId }
            
            if (optionIndex < 0) {
                return false
            }
            
            // Update option value
            val option = configOptions[optionIndex]
            configOptions[optionIndex] = option.copy(currentValue = value)
            
            // Update plugin with new config options
            currentPlugins[index] = plugin.copy(configOptions = configOptions)
            _installedPlugins.value = currentPlugins
            
            // Save changes
            saveInstalledPlugins()
            
            Log.d(TAG, "Updated plugin config: ${plugin.name}, option: $optionId, value: $value")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to update plugin config", e)
            return false
        }
    }
    
    /**
     * Update plugin state data
     * 
     * @param pluginId ID of the plugin
     * @param stateData New state data for the plugin
     * @return True if successful, false otherwise
     */
    fun updatePluginStateData(pluginId: String, stateData: Map<String, String>): Boolean {
        try {
            // Update plugin state
            val currentStates = _pluginStates.value.toMutableMap()
            val state = currentStates[pluginId] ?: PluginState(pluginId = pluginId)
            currentStates[pluginId] = state.copy(stateData = stateData)
            _pluginStates.value = currentStates
            
            // Save changes
            savePluginStates()
            
            Log.d(TAG, "Updated plugin state data: $pluginId")
            return true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to update plugin state data", e)
            return false
        }
    }
    
    /**
     * Get a plugin by ID
     * 
     * @param pluginId ID of the plugin
     * @return The plugin, or null if not found
     */
    fun getPlugin(pluginId: String): Plugin? {
        return _installedPlugins.value.find { it.id == pluginId }
    }
    
    /**
     * Get a plugin state by plugin ID
     * 
     * @param pluginId ID of the plugin
     * @return The plugin state, or null if not found
     */
    fun getPluginState(pluginId: String): PluginState? {
        return _pluginStates.value[pluginId]
    }
    
    /**
     * Get all enabled plugins
     * 
     * @return List of enabled plugins
     */
    fun getEnabledPlugins(): List<Plugin> {
        return _installedPlugins.value.filter { it.enabled }
    }
} 