package com.eb.obd2.viewmodels

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.eb.obd2.models.CapabilityType
import com.eb.obd2.models.ConfigOptionType
import com.eb.obd2.models.Plugin
import com.eb.obd2.models.PluginConfigOption
import com.eb.obd2.models.PluginState
import com.eb.obd2.repositories.PluginRepository
import com.eb.obd2.services.PluginService
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import javax.inject.Inject

/**
 * ViewModel for plugin management
 */
@HiltViewModel
class PluginViewModel @Inject constructor(
    private val pluginRepository: PluginRepository,
    private val pluginService: PluginService
) : ViewModel() {
    
    // Available plugins from server
    val availablePlugins = pluginRepository.availablePlugins
    
    // Installed plugins
    val installedPlugins = pluginRepository.installedPlugins
    
    // Plugin states
    val pluginStates = pluginRepository.pluginStates
    
    // Plugin visualizations
    val pluginVisualizations = pluginService.pluginVisualizations
    
    // Plugin commands
    val pluginCommands = pluginService.pluginCommands
    
    // Server URL
    private val _serverUrl = MutableStateFlow(pluginRepository.getPluginServerUrl())
    val serverUrl: StateFlow<String> = _serverUrl.asStateFlow()
    
    // Loading state
    private val _isLoading = MutableStateFlow(false)
    val isLoading: StateFlow<Boolean> = _isLoading.asStateFlow()
    
    // Error message
    private val _errorMessage = MutableStateFlow<String?>(null)
    val errorMessage: StateFlow<String?> = _errorMessage.asStateFlow()
    
    // Selected plugin for detail view
    private val _selectedPlugin = MutableStateFlow<Plugin?>(null)
    val selectedPlugin: StateFlow<Plugin?> = _selectedPlugin.asStateFlow()
    
    // Plugin installation progress
    private val _installProgress = MutableStateFlow<Pair<String, Float>?>(null)
    val installProgress: StateFlow<Pair<String, Float>?> = _installProgress.asStateFlow()
    
    // Filter by capability
    private val _capabilityFilter = MutableStateFlow<CapabilityType?>(null)
    val capabilityFilter: StateFlow<CapabilityType?> = _capabilityFilter.asStateFlow()
    
    init {
        // Initial fetch of available plugins
        fetchAvailablePlugins()
    }
    
    /**
     * Set the server URL
     * 
     * @param url The server URL
     */
    fun setServerUrl(url: String) {
        pluginRepository.setPluginServerUrl(url)
        _serverUrl.value = url
    }
    
    /**
     * Reset the server URL to default
     */
    fun resetServerUrl() {
        pluginRepository.resetPluginServerUrl()
        _serverUrl.value = pluginRepository.getPluginServerUrl()
    }
    
    /**
     * Fetch available plugins from the server
     */
    fun fetchAvailablePlugins() {
        viewModelScope.launch {
            _isLoading.value = true
            _errorMessage.value = null
            
            val success = pluginRepository.fetchAvailablePlugins()
            
            if (!success) {
                _errorMessage.value = "Failed to fetch plugins from server"
            }
            
            _isLoading.value = false
        }
    }
    
    /**
     * Install a plugin
     * 
     * @param pluginId ID of the plugin to install
     */
    fun installPlugin(pluginId: String) {
        viewModelScope.launch {
            _isLoading.value = true
            _errorMessage.value = null
            _installProgress.value = Pair(pluginId, 0.0f)
            
            val success = pluginRepository.installPlugin(pluginId)
            
            if (!success) {
                _errorMessage.value = "Failed to install plugin"
            }
            
            _installProgress.value = null
            _isLoading.value = false
        }
    }
    
    /**
     * Uninstall a plugin
     * 
     * @param pluginId ID of the plugin to uninstall
     */
    fun uninstallPlugin(pluginId: String) {
        viewModelScope.launch {
            _isLoading.value = true
            _errorMessage.value = null
            
            val success = pluginRepository.uninstallPlugin(pluginId)
            
            if (!success) {
                _errorMessage.value = "Failed to uninstall plugin"
            }
            
            _isLoading.value = false
        }
    }
    
    /**
     * Enable or disable a plugin
     * 
     * @param pluginId ID of the plugin
     * @param enabled Whether to enable or disable the plugin
     */
    fun setPluginEnabled(pluginId: String, enabled: Boolean) {
        viewModelScope.launch {
            _isLoading.value = true
            _errorMessage.value = null
            
            val success = pluginRepository.setPluginEnabled(pluginId, enabled)
            
            if (!success) {
                _errorMessage.value = "Failed to ${if (enabled) "enable" else "disable"} plugin"
            }
            
            _isLoading.value = false
        }
    }
    
    /**
     * Update a plugin configuration option
     * 
     * @param pluginId ID of the plugin
     * @param optionId ID of the configuration option
     * @param value New value for the option
     */
    fun updatePluginConfig(pluginId: String, optionId: String, value: String) {
        viewModelScope.launch {
            _isLoading.value = true
            _errorMessage.value = null
            
            val success = pluginRepository.updatePluginConfig(pluginId, optionId, value)
            
            if (!success) {
                _errorMessage.value = "Failed to update plugin configuration"
            }
            
            _isLoading.value = false
        }
    }
    
    /**
     * Select a plugin for detail view
     * 
     * @param pluginId ID of the plugin to select, or null to clear selection
     */
    fun selectPlugin(pluginId: String?) {
        if (pluginId == null) {
            _selectedPlugin.value = null
            return
        }
        
        viewModelScope.launch {
            // Try to find in installed plugins first
            val installedPlugin = installedPlugins.value.find { it.id == pluginId }
            if (installedPlugin != null) {
                _selectedPlugin.value = installedPlugin
                return@launch
            }
            
            // If not found, look in available plugins
            val availablePlugin = availablePlugins.value.find { it.id == pluginId }
            if (availablePlugin != null) {
                _selectedPlugin.value = availablePlugin
                return@launch
            }
            
            // Not found in either list
            _selectedPlugin.value = null
        }
    }
    
    /**
     * Set capability filter
     * 
     * @param capability The capability to filter by, or null to clear filter
     */
    fun setCapabilityFilter(capability: CapabilityType?) {
        _capabilityFilter.value = capability
    }
    
    /**
     * Get filtered installed plugins
     * 
     * @return List of plugins filtered by the current capability filter
     */
    fun getFilteredInstalledPlugins(): List<Plugin> {
        val capability = _capabilityFilter.value ?: return installedPlugins.value
        
        return installedPlugins.value.filter { plugin ->
            plugin.capabilities.any { it.type == capability }
        }
    }
    
    /**
     * Get filtered available plugins
     * 
     * @return List of plugins filtered by the current capability filter
     */
    fun getFilteredAvailablePlugins(): List<Plugin> {
        val capability = _capabilityFilter.value ?: return availablePlugins.value
        
        return availablePlugins.value.filter { plugin ->
            plugin.capabilities.any { it.type == capability }
        }
    }
    
    /**
     * Execute a command on a plugin
     * 
     * @param pluginId ID of the plugin
     * @param command Command to execute
     * @param parameters Command parameters
     * @return Command result
     */
    fun executeCommand(
        pluginId: String,
        command: String,
        parameters: Map<String, String> = emptyMap()
    ): Map<String, Any>? {
        return pluginService.executeCommand(pluginId, command, parameters)
    }
    
    /**
     * Clear error message
     */
    fun clearError() {
        _errorMessage.value = null
    }

    /**
     * Get the plugin service for views that need direct access
     */
    fun getPluginService(): PluginService {
        return pluginService
    }
} 