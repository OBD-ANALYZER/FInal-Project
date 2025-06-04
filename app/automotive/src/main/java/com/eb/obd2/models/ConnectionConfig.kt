package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Connection configuration for the OBD emulator
 */
@Serializable
data class ConnectionConfig(
    /** Unique identifier for this configuration */
    val id: String,
    
    /** User-friendly name for this configuration */
    val name: String,
    
    /** Connection host address (IP or hostname) */
    val host: String,
    
    /** Connection port */
    val port: Int,
    
    /** Use Bluetooth instead of TCP/IP */
    val useBluetooth: Boolean = false,
    
    /** Bluetooth device address (MAC) if using Bluetooth */
    val bluetoothAddress: String? = null,
    
    /** Auto-reconnect on connection loss */
    val autoReconnect: Boolean = true,
    
    /** Maximum reconnection attempts */
    val maxReconnectAttempts: Int = 3,
    
    /** Reconnection delay in milliseconds */
    val reconnectDelayMs: Int = 5000,
    
    /** Is this the default connection */
    val isDefault: Boolean = false,
    
    /** Additional connection parameters */
    val additionalParams: Map<String, String> = emptyMap()
) {
    companion object {
        /** Create a default TCP/IP connection configuration */
        fun createDefault(): ConnectionConfig = ConnectionConfig(
            id = "default",
            name = "Default Connection",
            host = "127.0.0.1",
            port = 35000,
            autoReconnect = true,
            isDefault = true
        )
        
        /** Create a default Bluetooth connection configuration */
        fun createDefaultBluetooth(): ConnectionConfig = ConnectionConfig(
            id = "default_bt",
            name = "Default Bluetooth",
            host = "",
            port = 0,
            useBluetooth = true,
            bluetoothAddress = null,
            autoReconnect = true,
            isDefault = false
        )
    }
    
    /**
     * Create a copy with updated default status
     */
    fun withDefaultStatus(isDefault: Boolean): ConnectionConfig {
        return copy(isDefault = isDefault)
    }
    
    /**
     * Validate that the configuration has all required fields
     */
    fun isValid(): Boolean {
        return if (useBluetooth) {
            !bluetoothAddress.isNullOrBlank()
        } else {
            host.isNotBlank() && port > 0 && port <= 65535
        }
    }
}

/**
 * Configuration profile for a simulation scenario
 */
@Serializable
data class SimulationProfile(
    /** Unique identifier for this profile */
    val id: String,
    
    /** User-friendly name for this profile */
    val name: String,
    
    /** Description of this simulation profile */
    val description: String = "",
    
    /** Connection configuration to use */
    val connectionConfigId: String,
    
    /** Emulator settings */
    val emulatorSettings: Map<String, Float> = emptyMap(),
    
    /** Initial vehicle parameters */
    val initialParameters: Map<String, Float> = emptyMap(),
    
    /** Is this the default profile */
    val isDefault: Boolean = false,
    
    /** Profile thumbnail/icon resource ID */
    val iconResourceId: Int? = null
) {
    companion object {
        /** Create a default simulation profile */
        fun createDefault(connectionConfigId: String): SimulationProfile = SimulationProfile(
            id = "default",
            name = "Default Simulation",
            description = "Standard simulation with default parameters",
            connectionConfigId = connectionConfigId,
            emulatorSettings = mapOf(
                "engine_temp_factor" to 1.0f,
                "fuel_consumption_factor" to 1.0f,
                "rpm_response_factor" to 1.0f,
                "simulation_speed" to 1.0f
            ),
            isDefault = true
        )
    }
    
    /**
     * Create a copy with updated default status
     */
    fun withDefaultStatus(isDefault: Boolean): SimulationProfile {
        return copy(isDefault = isDefault)
    }
} 