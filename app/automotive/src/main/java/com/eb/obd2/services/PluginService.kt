package com.eb.obd2.services

import android.content.Context
import android.util.Log
import com.eb.obd2.models.CapabilityType
import com.eb.obd2.models.Plugin
import com.eb.obd2.repositories.PluginRepository
import dalvik.system.DexClassLoader
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Plugin loader interface that must be implemented by all plugins
 */
interface PluginLoader {
    /**
     * Initialize the plugin with context and configuration
     * 
     * @param context Application context
     * @param config Plugin configuration
     * @return True if initialization was successful, false otherwise
     */
    fun initialize(context: Context, config: Map<String, String>): Boolean
    
    /**
     * Get the plugin's version
     * 
     * @return Plugin version
     */
    fun getVersion(): String
    
    /**
     * Process OBD data
     * 
     * @param data The OBD data to process
     * @return Processed data
     */
    fun processData(data: Map<String, Any>): Map<String, Any>
    
    /**
     * Get the plugin's visualization components
     * 
     * @return Map of component name to component class name
     */
    fun getVisualizations(): Map<String, String>
    
    /**
     * Get the plugin's supported commands
     * 
     * @return Map of command name to command details
     */
    fun getSupportedCommands(): Map<String, String>
    
    /**
     * Execute a command
     * 
     * @param command Command to execute
     * @param parameters Command parameters
     * @return Command result
     */
    fun executeCommand(command: String, parameters: Map<String, String>): Map<String, Any>
    
    /**
     * Stop the plugin
     */
    fun stop()
}

/**
 * Service for managing OBD-II emulator plugins
 */
@Singleton
class PluginService @Inject constructor(
    private val context: Context,
    private val pluginRepository: PluginRepository
) {
    private val TAG = "PluginService"
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
    
    // Loaded plugin instances
    private val loadedPlugins = mutableMapOf<String, PluginLoader>()
    
    // Plugin class loaders
    private val pluginClassLoaders = mutableMapOf<String, DexClassLoader>()
    
    // Plugin visualization components
    private val _pluginVisualizations = MutableStateFlow<Map<String, Map<String, String>>>(emptyMap())
    val pluginVisualizations: StateFlow<Map<String, Map<String, String>>> = _pluginVisualizations.asStateFlow()
    
    // Plugin supported commands
    private val _pluginCommands = MutableStateFlow<Map<String, Map<String, String>>>(emptyMap())
    val pluginCommands: StateFlow<Map<String, Map<String, String>>> = _pluginCommands.asStateFlow()
    
    // Optimization directory for dex files
    private val dexOptDir by lazy {
        File(context.codeCacheDir, "dex-opt").apply {
            if (!exists()) mkdirs()
        }
    }
    
    init {
        // Monitor installed plugins and load/unload as needed
        scope.launch {
            pluginRepository.installedPlugins.collectLatest { plugins ->
                updateLoadedPlugins(plugins)
            }
        }
    }
    
    /**
     * Update loaded plugins based on currently installed plugins
     * 
     * @param plugins List of installed plugins
     */
    private fun updateLoadedPlugins(plugins: List<Plugin>) {
        val enabledPlugins = plugins.filter { it.enabled }
        val currentlyLoadedIds = loadedPlugins.keys.toSet()
        val enabledIds = enabledPlugins.map { it.id }.toSet()
        
        // Unload plugins that are no longer enabled
        currentlyLoadedIds.minus(enabledIds).forEach { pluginId ->
            unloadPlugin(pluginId)
        }
        
        // Load newly enabled plugins
        enabledIds.minus(currentlyLoadedIds).forEach { pluginId ->
            plugins.find { it.id == pluginId }?.let { plugin ->
                loadPlugin(plugin)
            }
        }
        
        // Update plugin visualizations and commands
        updatePluginVisualizations()
        updatePluginCommands()
    }
    
    /**
     * Load a plugin
     * 
     * @param plugin Plugin to load
     * @return True if successful, false otherwise
     */
    private fun loadPlugin(plugin: Plugin): Boolean {
        try {
            plugin.localPath?.let { path ->
                val pluginFile = File(path)
                if (!pluginFile.exists()) {
                    Log.e(TAG, "Plugin file not found: ${pluginFile.absolutePath}")
                    return false
                }
                
                // Create class loader for the plugin
                val classLoader = DexClassLoader(
                    pluginFile.absolutePath,
                    dexOptDir.absolutePath,
                    null,
                    javaClass.classLoader
                )
                
                // Look for the plugin loader class
                val loaderClass = classLoader.loadClass("com.eb.obd2.plugin.PluginLoaderImpl")
                
                // Create an instance of the plugin loader
                val pluginLoader = loaderClass.newInstance() as PluginLoader
                
                // Initialize the plugin
                val configMap = plugin.configOptions.associate { it.id to it.currentValue }
                val initResult = pluginLoader.initialize(context, configMap)
                
                if (!initResult) {
                    Log.e(TAG, "Failed to initialize plugin: ${plugin.name}")
                    return false
                }
                
                // Store the plugin loader and class loader
                loadedPlugins[plugin.id] = pluginLoader
                pluginClassLoaders[plugin.id] = classLoader
                
                Log.d(TAG, "Loaded plugin: ${plugin.name}")
                return true
            } ?: run {
                Log.e(TAG, "Plugin has no local path: ${plugin.name}")
                return false
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to load plugin: ${plugin.name}", e)
            return false
        }
    }
    
    /**
     * Unload a plugin
     * 
     * @param pluginId ID of the plugin to unload
     */
    private fun unloadPlugin(pluginId: String) {
        try {
            // Stop the plugin
            loadedPlugins[pluginId]?.stop()
            
            // Remove the plugin loader and class loader
            loadedPlugins.remove(pluginId)
            pluginClassLoaders.remove(pluginId)
            
            Log.d(TAG, "Unloaded plugin: $pluginId")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to unload plugin: $pluginId", e)
        }
    }
    
    /**
     * Update plugin visualizations
     */
    private fun updatePluginVisualizations() {
        val visualizations = mutableMapOf<String, Map<String, String>>()
        
        loadedPlugins.forEach { (pluginId, loader) ->
            try {
                visualizations[pluginId] = loader.getVisualizations()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to get visualizations for plugin: $pluginId", e)
            }
        }
        
        _pluginVisualizations.value = visualizations
    }
    
    /**
     * Update plugin commands
     */
    private fun updatePluginCommands() {
        val commands = mutableMapOf<String, Map<String, String>>()
        
        loadedPlugins.forEach { (pluginId, loader) ->
            try {
                commands[pluginId] = loader.getSupportedCommands()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to get commands for plugin: $pluginId", e)
            }
        }
        
        _pluginCommands.value = commands
    }
    
    /**
     * Process OBD data through all enabled plugins
     * 
     * @param data The OBD data to process
     * @return Map of plugin ID to processed data
     */
    fun processData(data: Map<String, Any>): Map<String, Map<String, Any>> {
        val result = mutableMapOf<String, Map<String, Any>>()
        
        loadedPlugins.forEach { (pluginId, loader) ->
            try {
                val processed = loader.processData(data)
                result[pluginId] = processed
            } catch (e: Exception) {
                Log.e(TAG, "Failed to process data with plugin: $pluginId", e)
                result[pluginId] = mapOf("error" to e.message.toString())
            }
        }
        
        return result
    }
    
    /**
     * Execute a command on a specific plugin
     * 
     * @param pluginId ID of the plugin to execute the command on
     * @param command Command to execute
     * @param parameters Command parameters
     * @return Command result, or null if plugin not found
     */
    fun executeCommand(
        pluginId: String, 
        command: String, 
        parameters: Map<String, String>
    ): Map<String, Any>? {
        return loadedPlugins[pluginId]?.let { loader ->
            try {
                loader.executeCommand(command, parameters)
            } catch (e: Exception) {
                Log.e(TAG, "Failed to execute command: $command on plugin: $pluginId", e)
                mapOf("error" to e.message.toString())
            }
        }
    }
    
    /**
     * Get all visualization components from all enabled plugins
     * 
     * @return Map of plugin ID to map of visualization name to class name
     */
    fun getAllVisualizations(): Map<String, Map<String, String>> {
        return _pluginVisualizations.value
    }
    
    /**
     * Get all supported commands from all enabled plugins
     * 
     * @return Map of plugin ID to map of command name to command details
     */
    fun getAllCommands(): Map<String, Map<String, String>> {
        return _pluginCommands.value
    }
    
    /**
     * Get plugins by capability
     * 
     * @param capability The capability type to filter by
     * @return List of plugins that have the specified capability
     */
    fun getPluginsByCapability(capability: CapabilityType): List<Plugin> {
        return pluginRepository.getEnabledPlugins().filter { plugin ->
            plugin.capabilities.any { it.type == capability }
        }
    }
    
    /**
     * Get the plugin loader for a specific plugin
     * 
     * @param pluginId ID of the plugin
     * @return The plugin loader, or null if not found
     */
    fun getPluginLoader(pluginId: String): PluginLoader? {
        return loadedPlugins[pluginId]
    }
} 