package com.eb.obd2.views.components

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Save
import androidx.compose.material.icons.outlined.FiberManualRecord
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FloatingActionButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.eb.obd2.models.AnimationCurve
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@Composable
fun AnimationCurveGraph(
    curve: AnimationCurve,
    progress: Float = 0f,
    modifier: Modifier = Modifier
) {
    val animatedProgress by animateFloatAsState(
        targetValue = progress,
        label = "curve_progress"
    )
    
    // Get MaterialTheme colors outside of Canvas scope
    val primaryColor = MaterialTheme.colorScheme.primary
    
    Canvas(
        modifier = modifier
            .height(160.dp)
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        val width = size.width
        val height = size.height
        val padding = 16f
        
        val effectiveWidth = width - (padding * 2)
        val effectiveHeight = height - (padding * 2)
        
        // Draw grid
        val dashPattern = floatArrayOf(5f, 5f)
        val gridColor = Color.Gray.copy(alpha = 0.3f)
        
        // Horizontal grid lines
        for (i in 0..4) {
            val y = padding + (effectiveHeight * i / 4)
            drawLine(
                color = gridColor,
                start = Offset(padding, y),
                end = Offset(width - padding, y),
                strokeWidth = 1f,
                pathEffect = PathEffect.dashPathEffect(dashPattern)
            )
        }
        
        // Vertical grid lines
        for (i in 0..4) {
            val x = padding + (effectiveWidth * i / 4)
            drawLine(
                color = gridColor,
                start = Offset(x, padding),
                end = Offset(x, height - padding),
                strokeWidth = 1f,
                pathEffect = PathEffect.dashPathEffect(dashPattern)
            )
        }
        
        // Draw axes
        drawLine(
            color = Color.Gray,
            start = Offset(padding, height - padding),
            end = Offset(width - padding, height - padding),
            strokeWidth = 2f
        )
        
        drawLine(
            color = Color.Gray,
            start = Offset(padding, height - padding),
            end = Offset(padding, padding),
            strokeWidth = 2f
        )
        
        // Draw curve points and line segments
        if (curve.points.isNotEmpty()) {
            val maxX = curve.duration()
            val maxY = maxOf(curve.maxSpeed(), 120f)
            
            var prevPoint: Offset? = null
            
            curve.points.forEach { point ->
                val x = padding + (point.x / maxX) * effectiveWidth
                val y = height - padding - (point.y / maxY) * effectiveHeight
                
                // Draw point
                drawCircle(
                    color = primaryColor,
                    radius = 4f,
                    center = Offset(x, y)
                )
                
                // Draw line segment
                prevPoint?.let { prev ->
                    drawLine(
                        color = primaryColor,
                        start = prev,
                        end = Offset(x, y),
                        strokeWidth = 2f
                    )
                }
                
                prevPoint = Offset(x, y)
            }
            
            // Draw progress indicator if progress > 0
            if (animatedProgress > 0f) {
                val progressX = padding + animatedProgress * effectiveWidth
                
                // Find the Y position by interpolating between points
                val sortedPoints = curve.points.sortedBy { it.x }
                val relativeX = animatedProgress * maxX
                
                var y1 = height - padding
                var y2 = y1
                
                // Find the two points the progress is between
                for (i in 0 until sortedPoints.size - 1) {
                    val p1 = sortedPoints[i]
                    val p2 = sortedPoints[i + 1]
                    
                    if (relativeX >= p1.x && relativeX <= p2.x) {
                        val t = (relativeX - p1.x) / (p2.x - p1.x)
                        val interpolatedY = p1.y + t * (p2.y - p1.y)
                        
                        y1 = height - padding - (p1.y / maxY) * effectiveHeight
                        y2 = height - padding - (p2.y / maxY) * effectiveHeight
                        val progressY = height - padding - (interpolatedY / maxY) * effectiveHeight
                        
                        // Draw a vertical line at the current progress
                        drawLine(
                            color = Color.Red,
                            start = Offset(progressX, height - padding),
                            end = Offset(progressX, progressY),
                            strokeWidth = 2f
                        )
                        
                        // Draw a circle at the current position on the curve
                        drawCircle(
                            color = Color.Red,
                            radius = 6f,
                            center = Offset(progressX, progressY)
                        )
                        
                        break
                    }
                }
            }
        }
    }
}

@Composable
fun ActionButtons(onPlayClick: () -> Unit, onDeleteClick: () -> Unit) {
    Row {
        IconButton(
            onClick = onPlayClick
        ) {
            Icon(
                imageVector = Icons.Default.PlayArrow,
                contentDescription = "Play",
                tint = MaterialTheme.colorScheme.primary
            )
        }
        
        IconButton(
            onClick = onDeleteClick
        ) {
            Icon(
                imageVector = Icons.Default.Delete,
                contentDescription = "Delete",
                tint = MaterialTheme.colorScheme.error
            )
        }
    }
}

@Composable
fun AnimationCurveItem(
    curve: AnimationCurve,
    onPlayClick: () -> Unit,
    onDeleteClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    val dateFormat = SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.getDefault())
    val dateStr = dateFormat.format(Date(curve.timestamp))
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = curve.name,
                    style = MaterialTheme.typography.titleMedium.copy(
                        fontWeight = FontWeight.Bold
                    ),
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.weight(1f)
                )
                
                ActionButtons(onPlayClick, onDeleteClick)
            }
            
            if (curve.description.isNotEmpty()) {
                Text(
                    text = curve.description,
                    style = MaterialTheme.typography.bodyMedium,
                    maxLines = 2,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
            
            AnimationCurveGraph(
                curve = curve,
                modifier = Modifier.padding(top = 16.dp)
            )
            
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(top = 8.dp),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "Duration: ${(curve.duration() / 1000).toInt()} seconds",
                    style = MaterialTheme.typography.bodySmall
                )
                
                Text(
                    text = "Created: $dateStr",
                    style = MaterialTheme.typography.bodySmall
                )
            }
        }
    }
}

@Composable
fun AnimationCurvesList(
    curves: List<AnimationCurve>,
    onPlayCurve: (AnimationCurve) -> Unit,
    onDeleteCurve: (AnimationCurve) -> Unit,
    modifier: Modifier = Modifier
) {
    if (curves.isEmpty()) {
        Box(
            modifier = modifier
                .fillMaxWidth()
                .padding(32.dp),
            contentAlignment = Alignment.Center
        ) {
            Text(
                text = "No saved animation curves yet",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.5f)
            )
        }
    } else {
        LazyColumn(
            modifier = modifier.fillMaxWidth()
        ) {
            items(curves) { curve ->
                AnimationCurveItem(
                    curve = curve,
                    onPlayClick = { onPlayCurve(curve) },
                    onDeleteClick = { onDeleteCurve(curve) }
                )
            }
        }
    }
}

@Composable
fun CurvePlaybackControls(
    isPlaying: Boolean,
    progress: Float,
    curveName: String,
    onPlayPause: () -> Unit,
    onStop: () -> Unit,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier
            .fillMaxWidth()
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = curveName,
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            
            Spacer(modifier = Modifier.height(8.dp))
            
            LinearProgressIndicator(
                progress = { progress },
                modifier = Modifier.fillMaxWidth(),
                color = MaterialTheme.colorScheme.primary,
                trackColor = MaterialTheme.colorScheme.surfaceVariant
            )
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // Inline implementation instead of using ButtonsRow
            Row(
                horizontalArrangement = Arrangement.Center,
                verticalAlignment = Alignment.CenterVertically
            ) {
                IconButton(
                    onClick = onPlayPause,
                    modifier = Modifier
                        .size(56.dp)
                        .border(
                            width = 1.dp,
                            color = MaterialTheme.colorScheme.primary,
                            shape = CircleShape
                        )
                ) {
                    Icon(
                        imageVector = if (isPlaying) Icons.Default.Pause else Icons.Default.PlayArrow,
                        contentDescription = if (isPlaying) "Pause" else "Play",
                        tint = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.size(32.dp)
                    )
                }
                
                Spacer(modifier = Modifier.width(24.dp))
                
                Button(onClick = onStop) {
                    Text(text = "Stop")
                }
            }
        }
    }
}

@Composable
fun RecordingControls(
    isRecording: Boolean,
    recordingMessage: String,
    onStartRecording: () -> Unit,
    onStopRecording: () -> Unit,
    modifier: Modifier = Modifier
) {
    val recordingColor = if (isRecording) Color.Red else MaterialTheme.colorScheme.primary
    
    Card(
        modifier = modifier
            .fillMaxWidth()
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Record Driving Pattern",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold
            )
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(40.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = if (isRecording) recordingMessage else "Press record to start capturing a driving pattern",
                    style = MaterialTheme.typography.bodyMedium,
                    color = if (isRecording) recordingColor else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            // Inline implementation instead of using RecordingButtonsRow
            if (isRecording) {
                Row(
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    FloatingActionButton(
                        onClick = onStopRecording,
                        containerColor = recordingColor,
                        contentColor = Color.White
                    ) {
                        Icon(
                            imageVector = Icons.Default.Save,
                            contentDescription = "Stop Recording",
                            modifier = Modifier.size(24.dp)
                        )
                    }
                    
                    Spacer(modifier = Modifier.width(16.dp))
                    
                    Text(
                        text = "Stop & Save",
                        style = MaterialTheme.typography.bodyLarge,
                        fontWeight = FontWeight.Bold,
                        color = recordingColor
                    )
                }
            } else {
                Row(
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    FloatingActionButton(
                        onClick = onStartRecording,
                        containerColor = recordingColor,
                        contentColor = Color.White
                    ) {
                        Icon(
                            imageVector = Icons.Outlined.FiberManualRecord,
                            contentDescription = "Start Recording",
                            modifier = Modifier.size(24.dp)
                        )
                    }
                    
                    Spacer(modifier = Modifier.width(16.dp))
                    
                    Text(
                        text = "Start Recording",
                        style = MaterialTheme.typography.bodyLarge,
                        fontWeight = FontWeight.Bold,
                        color = recordingColor
                    )
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SavePatternDialog(
    onDismiss: () -> Unit,
    onSave: (String, String) -> Unit,
) {
    var name by remember { mutableStateOf("Recorded Pattern") }
    var description by remember { mutableStateOf("") }
    
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Save Driving Pattern") },
        text = {
            Column {
                OutlinedTextField(
                    value = name,
                    onValueChange = { name = it },
                    label = { Text("Pattern Name") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
                
                Spacer(modifier = Modifier.height(16.dp))
                
                OutlinedTextField(
                    value = description,
                    onValueChange = { description = it },
                    label = { Text("Description (Optional)") },
                    maxLines = 3,
                    modifier = Modifier.fillMaxWidth()
                )
            }
        },
        confirmButton = {
            TextButton(
                onClick = {
                    if (name.isNotBlank()) {
                        onSave(name, description)
                    }
                }
            ) {
                Text("Save")
            }
        },
        dismissButton = {
            TextButton(onClick = onDismiss) {
                Text("Cancel")
            }
        }
    )
}

@Composable
fun AnimationCurveManager(
    curves: List<AnimationCurve>,
    isPlaying: Boolean,
    isRecording: Boolean,
    curveProgress: Float,
    statusMessage: String,
    currentCurve: AnimationCurve?,
    onPlayCurve: (AnimationCurve) -> Unit,
    onStopPlayback: () -> Unit,
    onStartRecording: () -> Unit,
    onStopRecording: (String, String) -> Unit,
    onDeleteCurve: (AnimationCurve) -> Unit,
    modifier: Modifier = Modifier
) {
    var showSaveDialog by remember { mutableStateOf(false) }
    
    // Track recording stop action
    LaunchedEffect(isRecording) {
        if (!isRecording && statusMessage.contains("Pattern saved")) {
            showSaveDialog = false
        }
    }
    
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        // Section title
        Text(
            text = "Animation Curves",
            style = MaterialTheme.typography.titleLarge,
            color = MaterialTheme.colorScheme.primary,
            fontWeight = FontWeight.Bold
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Status message
        AnimatedVisibility(visible = statusMessage.isNotEmpty()) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 8.dp)
                    .background(
                        color = MaterialTheme.colorScheme.secondaryContainer,
                        shape = RoundedCornerShape(8.dp)
                    )
                    .padding(8.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = statusMessage,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onSecondaryContainer
                )
            }
        }
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Playback controls if currently playing
        AnimatedVisibility(visible = isPlaying && currentCurve != null) {
            currentCurve?.let {
                CurvePlaybackControls(
                    isPlaying = isPlaying,
                    progress = curveProgress,
                    curveName = it.name,
                    onPlayPause = {
                        // Toggle between play/pause (currently only stop is supported)
                        onStopPlayback()
                    },
                    onStop = onStopPlayback
                )
                
                Spacer(modifier = Modifier.height(16.dp))
            }
        }
        
        // Recording controls
        RecordingControls(
            isRecording = isRecording,
            recordingMessage = if (statusMessage.isNotEmpty()) statusMessage else "Recording in progress...",
            onStartRecording = onStartRecording,
            onStopRecording = {
                // Show save dialog when stopping recording
                showSaveDialog = true
            }
        )
        
        Spacer(modifier = Modifier.height(24.dp))
        
        // Saved curves list
        Text(
            text = "Saved Patterns",
            style = MaterialTheme.typography.titleMedium,
            fontWeight = FontWeight.Bold
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        AnimationCurvesList(
            curves = curves,
            onPlayCurve = onPlayCurve,
            onDeleteCurve = onDeleteCurve
        )
    }
    
    // Save pattern dialog
    if (showSaveDialog) {
        SavePatternDialog(
            onDismiss = {
                showSaveDialog = false
                // Cancel recording if dialog is dismissed
                if (isRecording) {
                    onStopRecording("Recorded Pattern", "")
                }
            },
            onSave = { name, description ->
                showSaveDialog = false
                onStopRecording(name, description)
            }
        )
    }
}

@Preview(showBackground = true)
@Composable
fun AnimationCurveGraphPreview() {
    AnimationCurveGraph(
        curve = AnimationCurve.createSample(),
        progress = 0.5f
    )
}

@Preview(showBackground = true)
@Composable
fun AnimationCurveItemPreview() {
    AnimationCurveItem(
        curve = AnimationCurve.createSample(),
        onPlayClick = {},
        onDeleteClick = {}
    )
}

@Preview(showBackground = true)
@Composable
fun CurvePlaybackControlsPreview() {
    CurvePlaybackControls(
        isPlaying = true,
        progress = 0.7f,
        curveName = "Sample Curve",
        onPlayPause = {},
        onStop = {}
    )
}

@Preview(showBackground = true)
@Composable
fun RecordingControlsPreview() {
    RecordingControls(
        isRecording = true,
        recordingMessage = "Recording in progress...",
        onStartRecording = {},
        onStopRecording = {}
    )
} 