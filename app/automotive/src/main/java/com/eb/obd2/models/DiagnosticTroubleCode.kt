package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Represents a Diagnostic Trouble Code (DTC) with its metadata
 */
@Serializable
data class DiagnosticTroubleCode(
    val code: String,
    val description: String,
    val severity: DTCSeverity = DTCSeverity.UNKNOWN,
    val category: DTCCategory = DTCCategory.UNKNOWN,
    val timestamp: Long = System.currentTimeMillis(),
    val isActive: Boolean = true,
    val hasBeenCleared: Boolean = false,
    val additionalInfo: Map<String, String> = emptyMap()
) {
    /**
     * Get the standard DTC prefix based on first character
     */
    fun getPrefix(): String {
        if (code.isEmpty()) return ""
        
        return when (code.first()) {
            'P' -> "Powertrain"
            'C' -> "Chassis"
            'B' -> "Body"
            'U' -> "Network"
            else -> "Unknown"
        }
    }
    
    /**
     * Check if the DTC is manufacturer-specific
     */
    fun isManufacturerSpecific(): Boolean {
        if (code.length < 2) return false
        
        // For standard DTCs, the second character is 0 or 1
        // For manufacturer-specific DTCs, it's typically 2 or 3
        return when (code[1]) {
            '0', '1' -> false
            '2', '3' -> true
            else -> false
        }
    }
    
    companion object {
        /**
         * Create a sample DTC for testing
         */
        fun createSample(): DiagnosticTroubleCode {
            return DiagnosticTroubleCode(
                code = "P0301",
                description = "Cylinder 1 Misfire Detected",
                severity = DTCSeverity.MODERATE,
                category = DTCCategory.ENGINE
            )
        }
        
        /**
         * Parse a DTC from its raw hex string
         */
        fun parseFromHex(hexCode: String): String {
            if (hexCode.length < 4) return "Unknown"
            
            try {
                // First two bytes contain the DTC prefix
                val firstByte = hexCode.substring(0, 2).toInt(16)
                
                // Determine the DTC type (P, C, B, U)
                val type = when ((firstByte and 0xC0) shr 6) {
                    0 -> "P"  // Powertrain
                    1 -> "C"  // Chassis
                    2 -> "B"  // Body
                    3 -> "U"  // Network
                    else -> "X" // Unknown
                }
                
                // Determine if standard or manufacturer-specific
                val subType = (firstByte and 0x30) shr 4
                
                // Remaining bits form the specific code
                val specific = (firstByte and 0x0F).toString(16) + 
                              hexCode.substring(2, 4)
                
                return "$type$subType$specific".uppercase()
            } catch (e: Exception) {
                return "Unknown code: $hexCode"
            }
        }
    }
}

/**
 * Severity level of a diagnostic trouble code
 */
@Serializable
enum class DTCSeverity {
    CRITICAL, // Immediate attention needed, potential safety issue
    SEVERE,   // Vehicle may not operate correctly, fix soon
    MODERATE, // Affects vehicle performance or emissions
    MINOR,    // Minor issue, should be addressed at next service
    INFO,     // Informational only
    UNKNOWN;  // Severity not classified
    
    fun getDescription(): String {
        return when (this) {
            CRITICAL -> "Critical - Immediate attention required"
            SEVERE -> "Severe - Fix as soon as possible"
            MODERATE -> "Moderate - Fix soon"
            MINOR -> "Minor - Fix at next service"
            INFO -> "Information only"
            UNKNOWN -> "Unknown severity"
        }
    }
}

/**
 * Category of the system affected by the DTC
 */
@Serializable
enum class DTCCategory {
    ENGINE,         // Engine control systems
    TRANSMISSION,   // Transmission/drivetrain
    EMISSIONS,      // Emissions control
    FUEL_SYSTEM,    // Fuel and air metering
    IGNITION,       // Ignition system
    AUXILIARY,      // Auxiliary emissions
    SPEED_CONTROL,  // Vehicle speed control
    COMPUTER,       // Computer and output circuits
    NETWORK,        // Network communications
    BODY,           // Body electrical
    CHASSIS,        // Chassis systems
    HVAC,           // Climate control
    RESTRAINTS,     // Safety restraint systems
    UNKNOWN;        // Not classified
    
    fun getDescription(): String {
        return when (this) {
            ENGINE -> "Engine Systems"
            TRANSMISSION -> "Transmission Systems"
            EMISSIONS -> "Emissions Control"
            FUEL_SYSTEM -> "Fuel and Air Metering"
            IGNITION -> "Ignition System"
            AUXILIARY -> "Auxiliary Emissions Controls"
            SPEED_CONTROL -> "Vehicle Speed Control"
            COMPUTER -> "Computer and Output Circuits"
            NETWORK -> "Network Communications"
            BODY -> "Body Electrical"
            CHASSIS -> "Chassis Systems"
            HVAC -> "Climate Control"
            RESTRAINTS -> "Safety Restraint Systems"
            UNKNOWN -> "Unclassified System"
        }
    }
} 