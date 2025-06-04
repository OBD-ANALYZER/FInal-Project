package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Represents a plugin that can be loaded into the OBD-II emulator
 */
@Serializable
data class Plugin(
    /** Unique identifier for this plugin */
    val id: String,
    
    /** User-friendly name for this plugin */
    val name: String,
    
    /** Description of this plugin */
    val description: String,
    
    /** Plugin version */
    val version: String,
    
    /** Author of the plugin */
    val author: String,
    
    /** Remote URL to download the plugin from */
    val downloadUrl: String,
    
    /** Whether the plugin is currently enabled */
    val enabled: Boolean = false,
    
    /** The plugin's compatibility version with the app */
    val compatibilityVersion: String,
    
    /** Plugin capabilities */
    val capabilities: List<PluginCapability> = emptyList(),
    
    /** Plugin configuration options */
    val configOptions: List<PluginConfigOption> = emptyList(),
    
    /** Last updated timestamp */
    val lastUpdated: Long = 0,
    
    /** Local installation path */
    val localPath: String? = null
)

/**
 * Represents a plugin capability
 */
@Serializable
data class PluginCapability(
    /** Unique identifier for this capability */
    val id: String,
    
    /** User-friendly name for this capability */
    val name: String,
    
    /** Description of this capability */
    val description: String,
    
    /** Type of capability */
    val type: CapabilityType,
    
    /** Required permissions */
    val requiredPermissions: List<String> = emptyList()
)

/**
 * Types of plugin capabilities
 */
@Serializable
enum class CapabilityType {
    DATA_PROCESSOR,
    VISUALIZATION,
    COMMAND_HANDLER,
    DIAGNOSTIC_TOOL,
    DATA_EXPORTER
}

/**
 * Represents a plugin configuration option
 */
@Serializable
data class PluginConfigOption(
    /** Unique identifier for this option */
    val id: String,
    
    /** User-friendly name for this option */
    val name: String,
    
    /** Description of this option */
    val description: String,
    
    /** Type of option */
    val type: ConfigOptionType,
    
    /** Default value */
    val defaultValue: String = "",
    
    /** Possible values (for selection-type options) */
    val possibleValues: List<String> = emptyList(),
    
    /** Current value */
    val currentValue: String = defaultValue,
    
    /** Whether this option is required */
    val required: Boolean = false
)

/**
 * Types of plugin configuration options
 */
@Serializable
enum class ConfigOptionType {
    STRING,
    NUMBER,
    BOOLEAN,
    SELECTION,
    MULTI_SELECTION,
    COLOR
}

/**
 * Represents the current state of a plugin
 */
@Serializable
data class PluginState(
    /** Plugin ID */
    val pluginId: String,
    
    /** Is the plugin currently active */
    val active: Boolean = false,
    
    /** Last execution time */
    val lastExecutionTime: Long = 0,
    
    /** Error message if any */
    val errorMessage: String? = null,
    
    /** Plugin-specific state data */
    val stateData: Map<String, String> = emptyMap()
) 