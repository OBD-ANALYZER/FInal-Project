package com.eb.obd2.views.components

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Folder
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import com.eb.obd2.viewmodels.OBDViewModel
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.concurrent.TimeUnit

/**
 * Control panel for data recording functionality
 */
@Composable
fun DataRecordingPanel(
    viewModel: OBDViewModel,
    modifier: Modifier = Modifier
) {
    val isRecording by viewModel.isRecording().collectAsState()
    val currentSessionName by viewModel.getCurrentSessionName().collectAsState()
    val recordingDuration by viewModel.getRecordingDuration().collectAsState()
    val availableSessions by viewModel.getRecordedSessions().collectAsState()
    
    var showSessionDialog by remember { mutableStateOf(false) }
    var showSessionsListDialog by remember { mutableStateOf(false) }
    var showDeleteConfirmDialog by remember { mutableStateOf(false) }
    var sessionToDelete by remember { mutableStateOf<String?>(null) }
    var customSessionName by remember { mutableStateOf("") }
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            // Title and enable switch
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Data Recording",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
                
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(
                        text = if (isRecording) "Recording" else "Enabled",
                        style = MaterialTheme.typography.labelMedium,
                        color = if (isRecording) 
                            Color(0xFFF44336) // Red
                        else 
                            MaterialTheme.colorScheme.onSurfaceVariant
                    )
                    
                    Spacer(modifier = Modifier.width(8.dp))
                    
                    Switch(
                        checked = true, // Always enabled in this implementation
                        onCheckedChange = { enabled ->
                            viewModel.setDataRecordingEnabled(enabled)
                        }
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // Recording status
            if (isRecording) {
                RecordingStatus(
                    sessionName = currentSessionName ?: "Unnamed Session",
                    durationSeconds = recordingDuration,
                    onStopRecording = {
                        viewModel.stopDataRecording()
                    }
                )
            } else {
                // Controls when not recording
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Button(
                        onClick = { 
                            customSessionName = ""
                            showSessionDialog = true 
                        }
                    ) {
                        Icon(
                            imageVector = Icons.Default.PlayArrow,
                            contentDescription = "Start Recording",
                            modifier = Modifier.size(20.dp)
                        )
                        
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        Text("Start Recording")
                    }
                    
                    Button(
                        onClick = { showSessionsListDialog = true },
                        enabled = availableSessions.isNotEmpty()
                    ) {
                        Icon(
                            imageVector = Icons.Default.Folder,
                            contentDescription = "Recorded Sessions",
                            modifier = Modifier.size(20.dp)
                        )
                        
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        Text("Recorded Sessions (${availableSessions.size})")
                    }
                }
            }
        }
    }
    
    // Session name dialog
    if (showSessionDialog) {
        AlertDialog(
            onDismissRequest = { showSessionDialog = false },
            title = { Text("New Recording Session") },
            text = {
                Column {
                    Text("Enter a name for this recording session or leave blank for an auto-generated name.")
                    
                    Spacer(modifier = Modifier.height(16.dp))
                    
                    OutlinedTextField(
                        value = customSessionName,
                        onValueChange = { customSessionName = it },
                        label = { Text("Session Name") },
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            },
            confirmButton = {
                Button(
                    onClick = {
                        val name = customSessionName.takeIf { it.isNotBlank() }
                        viewModel.startDataRecording(name)
                        showSessionDialog = false
                    }
                ) {
                    Text("Start")
                }
            },
            dismissButton = {
                TextButton(
                    onClick = { showSessionDialog = false }
                ) {
                    Text("Cancel")
                }
            }
        )
    }
    
    // Session list dialog
    if (showSessionsListDialog) {
        Dialog(onDismissRequest = { showSessionsListDialog = false }) {
            Card(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp)
            ) {
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(16.dp)
                ) {
                    Text(
                        text = "Recorded Sessions",
                        style = MaterialTheme.typography.titleLarge,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(modifier = Modifier.height(16.dp))
                    
                    if (availableSessions.isEmpty()) {
                        Text(
                            text = "No sessions recorded yet.",
                            style = MaterialTheme.typography.bodyMedium
                        )
                    } else {
                        LazyColumn(
                            modifier = Modifier
                                .fillMaxWidth()
                                .height(300.dp)
                        ) {
                            items(availableSessions) { session ->
                                SessionListItem(
                                    sessionName = session,
                                    onLoad = {
                                        // Load session for analysis
                                        val sessionData = viewModel.loadRecordedSession(session)
                                        // Process session data (in a real app, you would display this data)
                                        showSessionsListDialog = false
                                    },
                                    onDelete = {
                                        sessionToDelete = session
                                        showDeleteConfirmDialog = true
                                    }
                                )
                                
                                if (session != availableSessions.last()) {
                                    Spacer(modifier = Modifier.height(8.dp))
                                }
                            }
                        }
                    }
                    
                    Spacer(modifier = Modifier.height(16.dp))
                    
                    Button(
                        onClick = { showSessionsListDialog = false },
                        modifier = Modifier.align(Alignment.End)
                    ) {
                        Text("Close")
                    }
                }
            }
        }
    }
    
    // Delete confirmation dialog
    if (showDeleteConfirmDialog && sessionToDelete != null) {
        AlertDialog(
            onDismissRequest = { 
                showDeleteConfirmDialog = false
                sessionToDelete = null
            },
            title = { Text("Delete Session") },
            text = { Text("Are you sure you want to delete the session '$sessionToDelete'? This cannot be undone.") },
            confirmButton = {
                Button(
                    onClick = {
                        sessionToDelete?.let { session ->
                            viewModel.deleteRecordedSession(session)
                        }
                        showDeleteConfirmDialog = false
                        sessionToDelete = null
                    }
                ) {
                    Text("Delete")
                }
            },
            dismissButton = {
                TextButton(
                    onClick = { 
                        showDeleteConfirmDialog = false
                        sessionToDelete = null
                    }
                ) {
                    Text("Cancel")
                }
            }
        )
    }
}

/**
 * Display current recording status with timer
 */
@Composable
fun RecordingStatus(
    sessionName: String,
    durationSeconds: Long,
    onStopRecording: () -> Unit,
    modifier: Modifier = Modifier
) {
    val hours = TimeUnit.SECONDS.toHours(durationSeconds)
    val minutes = TimeUnit.SECONDS.toMinutes(durationSeconds) % 60
    val seconds = durationSeconds % 60
    
    val timeText = String.format("%02d:%02d:%02d", hours, minutes, seconds)
    
    Column(
        modifier = modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // Recording indicator
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Blinking recording indicator
            Box(
                modifier = Modifier
                    .size(12.dp)
                    .background(Color(0xFFF44336), RoundedCornerShape(6.dp))
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            Text(
                text = "Recording in progress...",
                style = MaterialTheme.typography.bodyMedium,
                fontWeight = FontWeight.Bold
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Session name
        Text(
            text = "Session: $sessionName",
            style = MaterialTheme.typography.bodyMedium
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Timer
        Text(
            text = timeText,
            style = MaterialTheme.typography.headlineMedium,
            fontWeight = FontWeight.Bold
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Stop button
        Button(
            onClick = onStopRecording
        ) {
            Icon(
                imageVector = Icons.Default.Stop,
                contentDescription = "Stop Recording",
                modifier = Modifier.size(20.dp)
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            Text("Stop Recording")
        }
    }
}

/**
 * List item for a recorded session
 */
@Composable
fun SessionListItem(
    sessionName: String,
    onLoad: () -> Unit,
    onDelete: () -> Unit,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier.fillMaxWidth()
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(8.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Session icon
            Icon(
                imageVector = Icons.Default.Folder,
                contentDescription = null,
                tint = MaterialTheme.colorScheme.primary,
                modifier = Modifier
                    .size(32.dp)
                    .padding(4.dp)
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            // Session info
            Column(
                modifier = Modifier
                    .weight(1f)
                    .clickable(onClick = onLoad)
            ) {
                Text(
                    text = sessionName,
                    style = MaterialTheme.typography.bodyLarge,
                    fontWeight = FontWeight.Bold
                )
                
                // Extract date from session name if possible
                val displayText = if (sessionName.startsWith("session_")) {
                    try {
                        val dateFormat = SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US)
                        val date = dateFormat.parse(sessionName.substring(8))
                        if (date != null) {
                            val displayFormat = SimpleDateFormat("MMM d, yyyy 'at' h:mm a", Locale.US)
                            displayFormat.format(date)
                        } else {
                            sessionName
                        }
                    } catch (e: Exception) {
                        // Couldn't parse date, just show the session name
                        sessionName
                    }
                } else {
                    sessionName
                }
                
                Text(
                    text = displayText,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            
            // Delete button
            IconButton(onClick = onDelete) {
                Icon(
                    imageVector = Icons.Default.Delete,
                    contentDescription = "Delete Session",
                    tint = Color(0xFFF44336)
                )
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun RecordingStatusPreview() {
    RecordingStatus(
        sessionName = "My Test Session",
        durationSeconds = 3725, // 1 hour, 2 minutes, 5 seconds
        onStopRecording = {}
    )
}

@Preview(showBackground = true)
@Composable
fun SessionListItemPreview() {
    SessionListItem(
        sessionName = "session_2023-05-15_14-30-22",
        onLoad = {},
        onDelete = {}
    )
} 