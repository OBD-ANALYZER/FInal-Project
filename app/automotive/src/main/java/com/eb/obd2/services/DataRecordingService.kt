package com.eb.obd2.services

import android.content.Context
import android.util.Log
import com.eb.obd2.models.RuntimeRecord
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.io.File
import java.io.FileOutputStream
import java.io.PrintWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Service for recording and storing historical vehicle data
 */
@Singleton
class DataRecordingService @Inject constructor(
    private val context: Context
) {
    private val TAG = "DataRecordingService"
    
    // Directory for storing recorded data
    private val dataDirectory by lazy {
        File(context.filesDir, "vehicle_data").apply {
            if (!exists()) mkdirs()
        }
    }
    
    // Current recording session
    private var currentSession: RecordingSession? = null
    
    // Status of recording
    private val _isRecording = MutableStateFlow(false)
    val isRecording: StateFlow<Boolean> = _isRecording.asStateFlow()
    
    // Recording duration in seconds
    private val _recordingDuration = MutableStateFlow(0L)
    val recordingDuration: StateFlow<Long> = _recordingDuration.asStateFlow()
    
    // Whether data recording is enabled
    private val _isEnabled = MutableStateFlow(true)
    val isEnabled: StateFlow<Boolean> = _isEnabled.asStateFlow()
    
    // Session name
    private val _sessionName = MutableStateFlow<String?>(null)
    val sessionName: StateFlow<String?> = _sessionName.asStateFlow()
    
    // Available recorded sessions
    private val _availableSessions = MutableStateFlow<List<String>>(emptyList())
    val availableSessions: StateFlow<List<String>> = _availableSessions.asStateFlow()
    
    // Record buffer size
    private val recordBufferSize = 1000
    
    init {
        // Load available sessions
        refreshAvailableSessions()
    }
    
    /**
     * Start a new recording session
     */
    fun startRecording(name: String? = null) {
        if (_isRecording.value) {
            Log.w(TAG, "Recording already in progress")
            return
        }
        
        val sessionName = name ?: generateSessionName()
        _sessionName.value = sessionName
        
        val sessionFile = File(dataDirectory, "$sessionName.csv")
        currentSession = RecordingSession(sessionFile, CoroutineScope(Dispatchers.IO))
        
        _isRecording.value = true
        _recordingDuration.value = 0L
        
        Log.i(TAG, "Started recording session: $sessionName")
    }
    
    /**
     * Stop the current recording session
     */
    fun stopRecording() {
        if (!_isRecording.value) {
            Log.w(TAG, "No recording in progress")
            return
        }
        
        currentSession?.let {
            it.stop()
            currentSession = null
            _isRecording.value = false
            _recordingDuration.value = 0L
            _sessionName.value = null
            
            // Refresh available sessions
            refreshAvailableSessions()
            
            Log.i(TAG, "Stopped recording session")
        }
    }
    
    /**
     * Enable or disable data recording
     */
    fun setEnabled(enabled: Boolean) {
        _isEnabled.value = enabled
        
        if (!enabled && _isRecording.value) {
            stopRecording()
        }
    }
    
    /**
     * Record a data point
     */
    fun recordData(
        timestamp: Long,
        type: String,
        value: Float,
        additionalInfo: Map<String, String> = emptyMap()
    ) {
        if (!_isEnabled.value || !_isRecording.value || currentSession == null) {
            return
        }
        
        currentSession?.addDataPoint(DataPoint(timestamp, type, value, additionalInfo))
    }
    
    /**
     * Record an OBD data point
     */
    fun recordOBDData(record: RuntimeRecord) {
        if (!_isEnabled.value || !_isRecording.value || currentSession == null) {
            return
        }
        
        val timestamp = System.currentTimeMillis()
        val type = when (record) {
            is RuntimeRecord.PlainRecord -> record.command
            else -> "unknown"
        }
        val value = try {
            record.value.toFloat()
        } catch (e: Exception) {
            0.0f // Default value if conversion fails
        }
        
        val additionalInfo = mapOf(
            "unit" to record.unit,
            "raw" to record.rawValue
        )
        
        recordData(timestamp, type, value, additionalInfo)
    }
    
    /**
     * Load a recorded session
     */
    fun loadSession(sessionName: String): Map<String, List<DataPoint>> {
        val sessionFile = File(dataDirectory, "$sessionName.csv")
        if (!sessionFile.exists()) {
            Log.e(TAG, "Session file does not exist: $sessionName")
            return emptyMap()
        }
        
        val result = mutableMapOf<String, MutableList<DataPoint>>()
        
        try {
            sessionFile.bufferedReader().useLines { lines ->
                // Skip header line
                lines.drop(1).forEach { line ->
                    try {
                        val parts = line.split(",")
                        if (parts.size >= 4) {
                            val timestamp = parts[0].toLongOrNull() ?: 0L
                            val type = parts[1]
                            val value = parts[2].toFloatOrNull() ?: 0.0f
                            val additionalInfo = parts.getOrNull(3)?.let {
                                parseAdditionalInfo(it)
                            } ?: emptyMap()
                            
                            val dataPoint = DataPoint(timestamp, type, value, additionalInfo)
                            result.getOrPut(type) { mutableListOf() }.add(dataPoint)
                        }
                    } catch (e: Exception) {
                        Log.e(TAG, "Error parsing line: $line", e)
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error loading session data", e)
        }
        
        return result
    }
    
    /**
     * Delete a recorded session
     */
    fun deleteSession(sessionName: String): Boolean {
        val sessionFile = File(dataDirectory, "$sessionName.csv")
        if (!sessionFile.exists()) {
            Log.e(TAG, "Session file does not exist: $sessionName")
            return false
        }
        
        val result = sessionFile.delete()
        refreshAvailableSessions()
        return result
    }
    
    /**
     * Refresh the list of available sessions
     */
    private fun refreshAvailableSessions() {
        val sessions = dataDirectory.listFiles()
            ?.filter { it.isFile && it.extension == "csv" }
            ?.map { it.nameWithoutExtension }
            ?.sorted()
            ?: emptyList()
        
        _availableSessions.value = sessions
    }
    
    /**
     * Generate a session name based on the current date and time
     */
    private fun generateSessionName(): String {
        val dateFormat = SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
        return "session_${dateFormat.format(Date())}"
    }
    
    /**
     * Parse additional info from a string
     */
    private fun parseAdditionalInfo(infoString: String): Map<String, String> {
        return try {
            infoString.split(";").associate { keyValue ->
                val parts = keyValue.split("=")
                if (parts.size == 2) {
                    parts[0] to parts[1]
                } else {
                    "" to ""
                }
            }.filterKeys { it.isNotEmpty() }
        } catch (e: Exception) {
            Log.e(TAG, "Error parsing additional info: $infoString", e)
            emptyMap()
        }
    }
    
    /**
     * Representation of a data point
     */
    data class DataPoint(
        val timestamp: Long,
        val type: String,
        val value: Float,
        val additionalInfo: Map<String, String> = emptyMap()
    ) {
        /**
         * Convert to CSV format
         */
        fun toCsvString(): String {
            val additionalInfoString = additionalInfo.entries.joinToString(";") { (key, value) ->
                "$key=$value"
            }
            
            return "$timestamp,$type,$value,$additionalInfoString"
        }
    }
    
    /**
     * Representation of a recording session
     */
    private inner class RecordingSession(
        private val file: File,
        private val scope: CoroutineScope
    ) {
        private val writer: PrintWriter
        private val buffer = mutableListOf<DataPoint>()
        private var durationJob: Job? = null
        private var flushJob: Job? = null
        private val startTime = System.currentTimeMillis()
        
        init {
            // Create file and write header
            writer = PrintWriter(FileOutputStream(file, false))
            writer.println("timestamp,type,value,additionalInfo")
            
            // Start duration counter
            durationJob = scope.launch {
                while (true) {
                    _recordingDuration.value = (System.currentTimeMillis() - startTime) / 1000
                    delay(1000)
                }
            }
            
            // Start periodic flush
            flushJob = scope.launch {
                while (true) {
                    delay(5000) // Flush every 5 seconds
                    flushBuffer()
                }
            }
        }
        
        /**
         * Add a data point to the buffer
         */
        fun addDataPoint(dataPoint: DataPoint) {
            synchronized(buffer) {
                buffer.add(dataPoint)
                
                // Flush if buffer is full
                if (buffer.size >= recordBufferSize) {
                    scope.launch {
                        flushBuffer()
                    }
                }
            }
        }
        
        /**
         * Flush the buffer to the file
         */
        private fun flushBuffer() {
            synchronized(buffer) {
                if (buffer.isNotEmpty()) {
                    buffer.forEach { dataPoint ->
                        writer.println(dataPoint.toCsvString())
                    }
                    writer.flush()
                    buffer.clear()
                }
            }
        }
        
        /**
         * Stop the recording session
         */
        fun stop() {
            durationJob?.cancel()
            flushJob?.cancel()
            
            // Flush any remaining data
            flushBuffer()
            
            // Close the writer
            writer.close()
        }
    }
} 