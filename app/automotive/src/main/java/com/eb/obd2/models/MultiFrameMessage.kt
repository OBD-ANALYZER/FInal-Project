package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Represents a multi-frame OBD message that spans multiple response frames
 */
@Serializable
data class MultiFrameMessage(
    val frames: MutableList<Frame> = mutableListOf(),
    val protocolType: ProtocolType = ProtocolType.AUTO,
    val isComplete: Boolean = false,
    val timestamp: Long = System.currentTimeMillis(),
    val expectedLength: Int = 0
) {
    /**
     * Get the assembled payload from all frames
     */
    fun getAssembledPayload(): String {
        if (frames.isEmpty()) return ""
        
        // Sort frames by sequence number
        val sortedFrames = frames.sortedBy { it.sequenceNumber }
        
        // Assemble the payload
        val stringBuilder = StringBuilder()
        sortedFrames.forEach { frame ->
            // Skip header bytes and include only data payload
            stringBuilder.append(frame.data)
        }
        
        return stringBuilder.toString()
    }
    
    /**
     * Add a new frame to the message
     */
    fun addFrame(frame: Frame): Boolean {
        // Check if this frame is already in the list
        if (frames.any { it.sequenceNumber == frame.sequenceNumber }) {
            return false
        }
        
        frames.add(frame)
        return true
    }
    
    /**
     * Check if all expected frames have been received
     */
    fun checkCompleteness(): Boolean {
        if (expectedLength <= 0) return false
        
        // For standard multi-frame messages, check sequence numbers
        val sequenceNumbers = frames.map { it.sequenceNumber }.toSet()
        return sequenceNumbers.size == expectedLength
    }
    
    @Serializable
    data class Frame(
        val sequenceNumber: Int,
        val data: String,
        val timestamp: Long = System.currentTimeMillis(),
        val isValid: Boolean = true
    )
}

/**
 * Represents a UDS (Unified Diagnostic Services) Request
 */
@Serializable
data class UDSRequest(
    val serviceId: UDSServiceID,
    val subFunction: Int? = null,
    val dataParameters: ByteArray = byteArrayOf(),
    val responseRequired: Boolean = true,
    val extendedSession: Boolean = false
) {
    /**
     * Convert UDS request to byte array for transmission
     */
    fun toByteArray(): ByteArray {
        val result = mutableListOf<Byte>()
        
        // Add service ID
        result.add(serviceId.id.toByte())
        
        // Add sub-function if present
        subFunction?.let {
            result.add(it.toByte())
        }
        
        // Add data parameters
        dataParameters.forEach { result.add(it) }
        
        return result.toByteArray()
    }
    
    /**
     * Format the request as a hex string
     */
    fun toHexString(): String {
        return toByteArray().joinToString("") { 
            String.format("%02X", it) 
        }
    }
    
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as UDSRequest

        if (serviceId != other.serviceId) return false
        if (subFunction != other.subFunction) return false
        if (!dataParameters.contentEquals(other.dataParameters)) return false
        if (responseRequired != other.responseRequired) return false
        if (extendedSession != other.extendedSession) return false

        return true
    }

    override fun hashCode(): Int {
        var result = serviceId.hashCode()
        result = 31 * result + (subFunction ?: 0)
        result = 31 * result + dataParameters.contentHashCode()
        result = 31 * result + responseRequired.hashCode()
        result = 31 * result + extendedSession.hashCode()
        return result
    }
}

/**
 * Represents a UDS (Unified Diagnostic Services) Response
 */
@Serializable
data class UDSResponse(
    val responseCode: Int,
    val serviceId: UDSServiceID,
    val data: ByteArray = byteArrayOf(),
    val isPositiveResponse: Boolean = true,
    val negativeResponseCode: Int? = null
) {
    /**
     * Parse the response data based on service ID
     */
    fun parseData(): Map<String, String> {
        val result = mutableMapOf<String, String>()
        
        try {
            when (serviceId) {
                UDSServiceID.DIAGNOSTIC_SESSION_CONTROL -> {
                    if (data.isNotEmpty()) {
                        val sessionType = data[0].toInt() and 0xFF
                        result["sessionType"] = when (sessionType) {
                            1 -> "Default Session"
                            2 -> "Programming Session"
                            3 -> "Extended Diagnostic Session"
                            4 -> "Safety System Diagnostic Session"
                            else -> "Unknown Session ($sessionType)"
                        }
                    }
                }
                UDSServiceID.ECU_RESET -> {
                    if (data.isNotEmpty()) {
                        val resetType = data[0].toInt() and 0xFF
                        result["resetType"] = when (resetType) {
                            1 -> "Hard Reset"
                            2 -> "Key Off/On Reset"
                            3 -> "Soft Reset"
                            4 -> "Enable Rapid Power Shutdown"
                            5 -> "Disable Rapid Power Shutdown"
                            else -> "Unknown Reset Type ($resetType)"
                        }
                    }
                }
                UDSServiceID.READ_DTC_INFORMATION -> {
                    // Process DTC information
                    if (data.size >= 2) {
                        val dtcCount = data[0].toInt() and 0xFF
                        result["dtcCount"] = dtcCount.toString()
                        
                        // Process DTCs (each DTC is 2 bytes)
                        val dtcs = mutableListOf<String>()
                        for (i in 1 until data.size step 2) {
                            if (i + 1 < data.size) {
                                val dtcBytes = byteArrayOf(data[i], data[i + 1])
                                val dtcHex = dtcBytes.joinToString("") { 
                                    String.format("%02X", it) 
                                }
                                val dtcCode = DiagnosticTroubleCode.parseFromHex(dtcHex)
                                dtcs.add(dtcCode)
                            }
                        }
                        result["dtcs"] = dtcs.joinToString(", ")
                    }
                }
                UDSServiceID.CLEAR_DTC_INFORMATION -> {
                    result["result"] = "DTCs Cleared Successfully"
                }
                else -> {
                    // Generic data parsing for other services
                    result["rawData"] = data.joinToString(" ") { 
                        String.format("%02X", it) 
                    }
                }
            }
        } catch (e: Exception) {
            result["error"] = "Error parsing data: ${e.message}"
        }
        
        // Add negative response information if applicable
        if (!isPositiveResponse && negativeResponseCode != null) {
            result["negativeResponseCode"] = negativeResponseCode.toString()
            result["negativeResponseMeaning"] = getNegativeResponseMeaning(negativeResponseCode)
        }
        
        return result
    }
    
    /**
     * Get the meaning of a negative response code
     */
    private fun getNegativeResponseMeaning(code: Int): String {
        return when (code) {
            0x10 -> "General Reject"
            0x11 -> "Service Not Supported"
            0x12 -> "Sub-Function Not Supported"
            0x13 -> "Incorrect Message Length or Invalid Format"
            0x14 -> "Response Too Long"
            0x21 -> "Busy - Repeat Request"
            0x22 -> "Conditions Not Correct"
            0x24 -> "Request Sequence Error"
            0x25 -> "No Response From Sub-Network Component"
            0x26 -> "Failure Prevents Execution"
            0x31 -> "Request Out Of Range"
            0x33 -> "Security Access Denied"
            0x35 -> "Invalid Key"
            0x36 -> "Exceeded Number Of Attempts"
            0x37 -> "Required Time Delay Not Expired"
            0x70 -> "Upload/Download Not Accepted"
            0x71 -> "Transfer Data Suspended"
            0x72 -> "General Programming Failure"
            0x78 -> "Request Correctly Received - Response Pending"
            else -> "Unknown Negative Response Code"
        }
    }
    
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as UDSResponse

        if (responseCode != other.responseCode) return false
        if (serviceId != other.serviceId) return false
        if (!data.contentEquals(other.data)) return false
        if (isPositiveResponse != other.isPositiveResponse) return false
        if (negativeResponseCode != other.negativeResponseCode) return false

        return true
    }

    override fun hashCode(): Int {
        var result = responseCode
        result = 31 * result + serviceId.hashCode()
        result = 31 * result + data.contentHashCode()
        result = 31 * result + isPositiveResponse.hashCode()
        result = 31 * result + (negativeResponseCode ?: 0)
        return result
    }
}

/**
 * UDS Service IDs as defined in ISO 14229-1
 */
@Serializable
enum class UDSServiceID(val id: Int, val description: String) {
    DIAGNOSTIC_SESSION_CONTROL(0x10, "Diagnostic Session Control"),
    ECU_RESET(0x11, "ECU Reset"),
    SECURITY_ACCESS(0x27, "Security Access"),
    COMMUNICATION_CONTROL(0x28, "Communication Control"),
    TESTER_PRESENT(0x3E, "Tester Present"),
    ACCESS_TIMING_PARAMETERS(0x83, "Access Timing Parameters"),
    SECURED_DATA_TRANSMISSION(0x84, "Secured Data Transmission"),
    CONTROL_DTC_SETTINGS(0x85, "Control DTC Settings"),
    RESPONSE_ON_EVENT(0x86, "Response On Event"),
    LINK_CONTROL(0x87, "Link Control"),
    READ_DATA_BY_IDENTIFIER(0x22, "Read Data By Identifier"),
    READ_MEMORY_BY_ADDRESS(0x23, "Read Memory By Address"),
    READ_SCALING_DATA_BY_IDENTIFIER(0x24, "Read Scaling Data By Identifier"),
    READ_DATA_BY_PERIODIC_IDENTIFIER(0x2A, "Read Data By Periodic Identifier"),
    DYNAMICALLY_DEFINE_DATA_IDENTIFIER(0x2C, "Dynamically Define Data Identifier"),
    WRITE_DATA_BY_IDENTIFIER(0x2E, "Write Data By Identifier"),
    WRITE_MEMORY_BY_ADDRESS(0x3D, "Write Memory By Address"),
    CLEAR_DTC_INFORMATION(0x14, "Clear Diagnostic Information"),
    READ_DTC_INFORMATION(0x19, "Read DTC Information"),
    INPUT_OUTPUT_CONTROL_BY_IDENTIFIER(0x2F, "Input Output Control By Identifier"),
    ROUTINE_CONTROL(0x31, "Routine Control"),
    REQUEST_DOWNLOAD(0x34, "Request Download"),
    REQUEST_UPLOAD(0x35, "Request Upload"),
    TRANSFER_DATA(0x36, "Transfer Data"),
    REQUEST_TRANSFER_EXIT(0x37, "Request Transfer Exit"),
    UNKNOWN(0x00, "Unknown Service");
    
    companion object {
        /**
         * Get a UDSServiceID from its integer value
         */
        fun fromId(id: Int): UDSServiceID {
            return values().find { it.id == id } ?: UNKNOWN
        }
    }
}

/**
 * Protocol types supported by the OBD system
 */
@Serializable
enum class ProtocolType {
    AUTO,        // Automatic detection
    ISO_15765_4, // CAN (11-bit or 29-bit, 250 or 500 kbps)
    ISO_14230_4, // KWP2000 (5-baud init or fast init)
    ISO_9141_2,  // ISO 9141-2 (5-baud init)
    SAE_J1850_PWM, // SAE J1850 PWM (41.6 kbps)
    SAE_J1850_VPW, // SAE J1850 VPW (10.4 kbps)
    ISO_15765_4_29BIT, // CAN (29-bit ID, 250 kbps)
    ISO_15765_4_11BIT, // CAN (11-bit ID, 250 kbps)
    ISO_15765_4_29BIT_500K, // CAN (29-bit ID, 500 kbps)
    ISO_15765_4_11BIT_500K, // CAN (11-bit ID, 500 kbps)
    SAE_J1939;   // SAE J1939 (CAN 29-bit ID, 250 kbps)
    
    /**
     * Get a human-readable description of the protocol
     */
    fun getDescription(): String {
        return when (this) {
            AUTO -> "Auto"
            ISO_15765_4 -> "ISO 15765-4 CAN"
            ISO_14230_4 -> "ISO 14230-4 KWP2000"
            ISO_9141_2 -> "ISO 9141-2"
            SAE_J1850_PWM -> "SAE J1850 PWM"
            SAE_J1850_VPW -> "SAE J1850 VPW"
            ISO_15765_4_29BIT -> "ISO 15765-4 CAN (29-bit ID, 250 kbps)"
            ISO_15765_4_11BIT -> "ISO 15765-4 CAN (11-bit ID, 250 kbps)"
            ISO_15765_4_29BIT_500K -> "ISO 15765-4 CAN (29-bit ID, 500 kbps)"
            ISO_15765_4_11BIT_500K -> "ISO 15765-4 CAN (11-bit ID, 500 kbps)"
            SAE_J1939 -> "SAE J1939"
        }
    }
} 