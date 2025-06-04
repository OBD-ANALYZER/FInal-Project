package com.eb.obd2.repositories

import android.content.Context
import com.eb.obd2.models.DTCCategory
import com.eb.obd2.models.DTCSeverity
import com.eb.obd2.models.DiagnosticTroubleCode
import com.eb.obd2.models.UDSRequest
import com.eb.obd2.models.UDSResponse
import com.eb.obd2.models.UDSServiceID
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.random.Random

@Singleton
class DiagnosticRepository @Inject constructor(
    private val appContext: Context
) {
    private val _dtcCodes = MutableStateFlow<List<DiagnosticTroubleCode>>(emptyList())
    val dtcCodes: Flow<List<DiagnosticTroubleCode>> = _dtcCodes.asStateFlow()
    
    private val _isScanningActive = MutableStateFlow(false)
    val isScanningActive: Flow<Boolean> = _isScanningActive.asStateFlow()
    
    private val _scanProgress = MutableStateFlow(0f)
    val scanProgress: Flow<Float> = _scanProgress.asStateFlow()
    
    private val _scanMessage = MutableStateFlow("")
    val scanMessage: Flow<String> = _scanMessage.asStateFlow()
    
    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }
    private val dtcDirectory get() = File(appContext.filesDir, "dtc")
    
    // DTC database for known codes and descriptions - would be more extensive in production
    private val dtcDatabase = mapOf(
        "P0100" to Pair("Mass or Volume Air Flow Circuit Malfunction", DTCCategory.FUEL_SYSTEM),
        "P0101" to Pair("Mass or Volume Air Flow Circuit Range/Performance Problem", DTCCategory.FUEL_SYSTEM),
        "P0102" to Pair("Mass or Volume Air Flow Circuit Low Input", DTCCategory.FUEL_SYSTEM),
        "P0103" to Pair("Mass or Volume Air Flow Circuit High Input", DTCCategory.FUEL_SYSTEM),
        "P0105" to Pair("Manifold Absolute Pressure/Barometric Pressure Circuit Malfunction", DTCCategory.ENGINE),
        "P0106" to Pair("Manifold Absolute Pressure/Barometric Pressure Circuit Range/Performance Problem", DTCCategory.ENGINE),
        "P0107" to Pair("Manifold Absolute Pressure/Barometric Pressure Circuit Low Input", DTCCategory.ENGINE),
        "P0108" to Pair("Manifold Absolute Pressure/Barometric Pressure Circuit High Input", DTCCategory.ENGINE),
        "P0110" to Pair("Intake Air Temperature Circuit Malfunction", DTCCategory.ENGINE),
        "P0111" to Pair("Intake Air Temperature Circuit Range/Performance Problem", DTCCategory.ENGINE),
        "P0112" to Pair("Intake Air Temperature Circuit Low Input", DTCCategory.ENGINE),
        "P0113" to Pair("Intake Air Temperature Circuit High Input", DTCCategory.ENGINE),
        "P0115" to Pair("Engine Coolant Temperature Circuit Malfunction", DTCCategory.ENGINE),
        "P0116" to Pair("Engine Coolant Temperature Circuit Range/Performance Problem", DTCCategory.ENGINE),
        "P0117" to Pair("Engine Coolant Temperature Circuit Low Input", DTCCategory.ENGINE),
        "P0118" to Pair("Engine Coolant Temperature Circuit High Input", DTCCategory.ENGINE),
        "P0120" to Pair("Throttle Position Sensor/Switch A Circuit Malfunction", DTCCategory.FUEL_SYSTEM),
        "P0121" to Pair("Throttle Position Sensor/Switch A Circuit Range/Performance Problem", DTCCategory.FUEL_SYSTEM),
        "P0122" to Pair("Throttle Position Sensor/Switch A Circuit Low Input", DTCCategory.FUEL_SYSTEM),
        "P0123" to Pair("Throttle Position Sensor/Switch A Circuit High Input", DTCCategory.FUEL_SYSTEM),
        "P0130" to Pair("O2 Sensor Circuit Malfunction (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0131" to Pair("O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0132" to Pair("O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0133" to Pair("O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0134" to Pair("O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0135" to Pair("O2 Sensor Heater Circuit Malfunction (Bank 1 Sensor 1)", DTCCategory.EMISSIONS),
        "P0170" to Pair("Fuel Trim Malfunction (Bank 1)", DTCCategory.FUEL_SYSTEM),
        "P0171" to Pair("System Too Lean (Bank 1)", DTCCategory.FUEL_SYSTEM),
        "P0172" to Pair("System Too Rich (Bank 1)", DTCCategory.FUEL_SYSTEM),
        "P0175" to Pair("System Too Rich (Bank 2)", DTCCategory.FUEL_SYSTEM),
        "P0200" to Pair("Injector Circuit Malfunction", DTCCategory.FUEL_SYSTEM),
        "P0201" to Pair("Injector Circuit Malfunction - Cylinder 1", DTCCategory.FUEL_SYSTEM),
        "P0202" to Pair("Injector Circuit Malfunction - Cylinder 2", DTCCategory.FUEL_SYSTEM),
        "P0203" to Pair("Injector Circuit Malfunction - Cylinder 3", DTCCategory.FUEL_SYSTEM),
        "P0204" to Pair("Injector Circuit Malfunction - Cylinder 4", DTCCategory.FUEL_SYSTEM),
        "P0300" to Pair("Random/Multiple Cylinder Misfire Detected", DTCCategory.IGNITION),
        "P0301" to Pair("Cylinder 1 Misfire Detected", DTCCategory.IGNITION),
        "P0302" to Pair("Cylinder 2 Misfire Detected", DTCCategory.IGNITION),
        "P0303" to Pair("Cylinder 3 Misfire Detected", DTCCategory.IGNITION),
        "P0304" to Pair("Cylinder 4 Misfire Detected", DTCCategory.IGNITION),
        "P0305" to Pair("Cylinder 5 Misfire Detected", DTCCategory.IGNITION),
        "P0306" to Pair("Cylinder 6 Misfire Detected", DTCCategory.IGNITION),
        "P0400" to Pair("Exhaust Gas Recirculation Flow Malfunction", DTCCategory.EMISSIONS),
        "P0401" to Pair("Exhaust Gas Recirculation Flow Insufficient Detected", DTCCategory.EMISSIONS),
        "P0402" to Pair("Exhaust Gas Recirculation Flow Excessive Detected", DTCCategory.EMISSIONS)
    )
    
    init {
        // Ensure directory exists
        dtcDirectory.mkdirs()
        
        // Load saved DTCs
        loadSavedDTCs()
        
        // Load sample DTCs if none exist
        if (_dtcCodes.value.isEmpty()) {
            loadSampleDTCs()
        }
    }
    
    /**
     * Get all stored DTCs
     */
    suspend fun getAllDTCs(): List<DiagnosticTroubleCode> = withContext(Dispatchers.IO) {
        return@withContext _dtcCodes.value
    }
    
    /**
     * Get active DTCs (not cleared)
     */
    suspend fun getActiveDTCs(): List<DiagnosticTroubleCode> = withContext(Dispatchers.IO) {
        return@withContext _dtcCodes.value.filter { it.isActive && !it.hasBeenCleared }
    }
    
    /**
     * Get DTCs by category
     */
    suspend fun getDTCsByCategory(category: DTCCategory): List<DiagnosticTroubleCode> = withContext(Dispatchers.IO) {
        return@withContext _dtcCodes.value.filter { it.category == category }
    }
    
    /**
     * Get DTCs by severity
     */
    suspend fun getDTCsBySeverity(severity: DTCSeverity): List<DiagnosticTroubleCode> = withContext(Dispatchers.IO) {
        return@withContext _dtcCodes.value.filter { it.severity == severity }
    }
    
    /**
     * Save a DTC
     */
    suspend fun saveDTC(dtc: DiagnosticTroubleCode): Boolean = withContext(Dispatchers.IO) {
        try {
            val fileName = "${dtc.code}.json"
            val file = File(dtcDirectory, fileName)
            
            val jsonString = json.encodeToString(dtc)
            file.writeText(jsonString)
            
            // Update the saved DTCs list
            loadSavedDTCs()
            
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Clear a specific DTC
     */
    suspend fun clearDTC(dtcCode: String): Boolean = withContext(Dispatchers.IO) {
        try {
            val dtc = _dtcCodes.value.find { it.code == dtcCode } ?: return@withContext false
            
            // Create a new DTC with hasBeenCleared flag set to true
            val clearedDTC = dtc.copy(
                hasBeenCleared = true,
                isActive = false,
                timestamp = System.currentTimeMillis()
            )
            
            return@withContext saveDTC(clearedDTC)
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Clear all stored DTCs
     */
    suspend fun clearAllDTCs(): Boolean = withContext(Dispatchers.IO) {
        try {
            var allSuccess = true
            
            _dtcCodes.value.forEach { dtc ->
                val clearedDTC = dtc.copy(
                    hasBeenCleared = true,
                    isActive = false,
                    timestamp = System.currentTimeMillis()
                )
                
                val success = saveDTC(clearedDTC)
                if (!success) {
                    allSuccess = false
                }
            }
            
            return@withContext allSuccess
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Scan for diagnostic trouble codes
     */
    suspend fun scanForDTCs(): Boolean = withContext(Dispatchers.IO) {
        try {
            if (_isScanningActive.value) {
                return@withContext false
            }
            
            _isScanningActive.value = true
            _scanProgress.value = 0f
            _scanMessage.value = "Initializing scan..."
            
            // Simulate scanning process
            delay(500)
            _scanMessage.value = "Establishing communication with ECU..."
            _scanProgress.value = 0.1f
            
            delay(700)
            _scanMessage.value = "Reading Mode 03 (Stored DTCs)..."
            _scanProgress.value = 0.3f
            
            delay(800)
            _scanMessage.value = "Reading Mode 07 (Pending DTCs)..."
            _scanProgress.value = 0.5f
            
            delay(600)
            _scanMessage.value = "Reading Mode 0A (Permanent DTCs)..."
            _scanProgress.value = 0.7f
            
            delay(700)
            _scanMessage.value = "Processing results..."
            _scanProgress.value = 0.9f
            
            // Generate some random DTCs for demonstration
            val foundDTCs = generateRandomDTCs(Random.nextInt(0, 3))
            
            // Save the found DTCs
            foundDTCs.forEach { saveDTC(it) }
            
            delay(500)
            _scanMessage.value = "Scan complete. Found ${foundDTCs.size} DTCs."
            _scanProgress.value = 1.0f
            
            delay(1000)
            _isScanningActive.value = false
            _scanMessage.value = ""
            
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            _scanMessage.value = "Error: ${e.message}"
            _isScanningActive.value = false
            return@withContext false
        }
    }
    
    /**
     * Get DTC description, category, severity, etc. based on DTC code
     */
    fun getDTCMetadata(dtcCode: String): DiagnosticTroubleCode {
        // Check if we have this DTC in our database
        val (description, category) = dtcDatabase[dtcCode] ?: Pair(
            "Unknown DTC: $dtcCode", 
            DTCCategory.UNKNOWN
        )
        
        // Determine severity - in a real system this would be more sophisticated
        val severity = when {
            dtcCode.startsWith("P0") -> DTCSeverity.MODERATE // Generic powertrain codes
            dtcCode.startsWith("P1") -> DTCSeverity.MODERATE // Manufacturer-specific powertrain codes
            dtcCode.startsWith("P2") -> DTCSeverity.SEVERE   // Generic powertrain codes
            dtcCode.startsWith("P3") -> DTCSeverity.SEVERE   // Generic powertrain codes
            dtcCode.startsWith("C") -> DTCSeverity.CRITICAL  // Chassis codes (often safety-related)
            dtcCode.startsWith("B") -> DTCSeverity.MINOR     // Body codes (often comfort/convenience)
            dtcCode.startsWith("U") -> DTCSeverity.MODERATE  // Network codes
            else -> DTCSeverity.UNKNOWN
        }
        
        return DiagnosticTroubleCode(
            code = dtcCode,
            description = description,
            severity = severity,
            category = category,
            timestamp = System.currentTimeMillis(),
            isActive = true,
            hasBeenCleared = false
        )
    }
    
    /**
     * Convert a raw OBD response to a list of DTCs
     */
    fun parseDTCResponse(response: String): List<String> {
        val dtcs = mutableListOf<String>()
        
        try {
            // Remove spaces and split into pairs of bytes
            val cleaned = response.replace(" ", "")
            
            // Each DTC is represented by 2 bytes
            for (i in 0 until cleaned.length step 4) {
                if (i + 4 <= cleaned.length) {
                    val dtcBytes = cleaned.substring(i, i + 4)
                    
                    // Skip 0000 as it's typically used to indicate no DTCs
                    if (dtcBytes != "0000") {
                        val dtcCode = DiagnosticTroubleCode.parseFromHex(dtcBytes)
                        dtcs.add(dtcCode)
                    }
                }
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
        
        return dtcs
    }
    
    /**
     * Process a UDS response
     */
    suspend fun processUDSResponse(response: UDSResponse): List<DiagnosticTroubleCode> = withContext(Dispatchers.IO) {
        val result = mutableListOf<DiagnosticTroubleCode>()
        
        try {
            if (response.serviceId == UDSServiceID.READ_DTC_INFORMATION && response.isPositiveResponse) {
                val parsedData = response.parseData()
                
                // Extract DTCs from the parsed data
                val dtcsList = parsedData["dtcs"]?.split(", ") ?: emptyList()
                
                dtcsList.forEach { dtcCode ->
                    if (dtcCode.isNotEmpty()) {
                        val dtc = getDTCMetadata(dtcCode)
                        result.add(dtc)
                        saveDTC(dtc)
                    }
                }
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
        
        return@withContext result
    }
    
    /**
     * Create a UDS request to read DTCs
     */
    fun createReadDTCRequest(dtcType: DTCType = DTCType.STORED): UDSRequest {
        val subFunction = when (dtcType) {
            DTCType.STORED -> 0x01
            DTCType.PENDING -> 0x02
            DTCType.PERMANENT -> 0x03
            DTCType.CLEAR -> 0xFF
        }
        
        return UDSRequest(
            serviceId = if (dtcType == DTCType.CLEAR) 
                UDSServiceID.CLEAR_DTC_INFORMATION 
            else 
                UDSServiceID.READ_DTC_INFORMATION,
            subFunction = subFunction,
            responseRequired = true
        )
    }
    
    /**
     * Generate random DTCs for demonstration purposes
     */
    private fun generateRandomDTCs(count: Int): List<DiagnosticTroubleCode> {
        val result = mutableListOf<DiagnosticTroubleCode>()
        
        // Get a randomized subset of DTC codes from our database
        val availableCodes = dtcDatabase.keys.toList().shuffled()
        
        for (i in 0 until count) {
            if (i < availableCodes.size) {
                val code = availableCodes[i]
                result.add(getDTCMetadata(code))
            }
        }
        
        return result
    }
    
    /**
     * Load saved DTCs from storage
     */
    private fun loadSavedDTCs() {
        val codes = mutableListOf<DiagnosticTroubleCode>()
        
        dtcDirectory.listFiles()?.filter { it.isFile && it.extension == "json" }?.forEach { file ->
            try {
                val jsonString = file.readText()
                val dtc = json.decodeFromString<DiagnosticTroubleCode>(jsonString)
                codes.add(dtc)
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
        
        // Sort codes by timestamp (newest first)
        _dtcCodes.value = codes.sortedByDescending { it.timestamp }
    }
    
    /**
     * Load sample DTCs for demonstration
     */
    private fun loadSampleDTCs() {
        val sampleDTCs = listOf(
            DiagnosticTroubleCode(
                code = "P0171",
                description = "System Too Lean (Bank 1)",
                severity = DTCSeverity.MODERATE,
                category = DTCCategory.FUEL_SYSTEM,
                timestamp = System.currentTimeMillis() - 86400000, // 1 day ago
                isActive = true,
                hasBeenCleared = false
            ),
            DiagnosticTroubleCode(
                code = "P0300",
                description = "Random/Multiple Cylinder Misfire Detected",
                severity = DTCSeverity.SEVERE,
                category = DTCCategory.IGNITION,
                timestamp = System.currentTimeMillis() - 172800000, // 2 days ago
                isActive = true,
                hasBeenCleared = false
            ),
            DiagnosticTroubleCode(
                code = "P0401",
                description = "Exhaust Gas Recirculation Flow Insufficient Detected",
                severity = DTCSeverity.MINOR,
                category = DTCCategory.EMISSIONS,
                timestamp = System.currentTimeMillis() - 259200000, // 3 days ago
                isActive = false,
                hasBeenCleared = true
            )
        )
        
        // Save sample DTCs
        kotlinx.coroutines.runBlocking {
            sampleDTCs.forEach { saveDTC(it) }
        }
    }
}

/**
 * Type of DTCs to request
 */
enum class DTCType {
    STORED,     // Mode 03 - Stored DTCs (Confirmed)
    PENDING,    // Mode 07 - Pending DTCs (Not Confirmed)
    PERMANENT,  // Mode 0A - Permanent DTCs (Cannot be cleared)
    CLEAR       // Mode 04 - Clear DTCs
} 