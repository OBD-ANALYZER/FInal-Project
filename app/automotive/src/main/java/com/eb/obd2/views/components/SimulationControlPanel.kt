package com.eb.obd2.views.components

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.Sync
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp

@Composable
fun SimulationControlPanel(
    throttleValue: Float,
    onThrottleChange: (Float) -> Unit,
    brakeValue: Float,
    onBrakeChange: (Float) -> Unit,
    animationSpeed: Float,
    onAnimationSpeedChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    var showTwoWayInfo by remember { mutableStateOf(false) }
    
    Card(
        modifier = modifier
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            // Header with info button
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Simulation Controls",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                
                IconButton(onClick = { showTwoWayInfo = true }) {
                    Icon(
                        imageVector = Icons.Default.Info,
                        contentDescription = "Remote Control Info"
                    )
                }
            }
            
            // Throttle control slider
            Column(modifier = Modifier.padding(vertical = 8.dp)) {
                Text(
                    "Throttle: ${(throttleValue * 100).toInt()}%",
                    style = MaterialTheme.typography.bodyMedium
                )
                
                Slider(
                    value = throttleValue,
                    onValueChange = { newValue ->
                        // Automatically set brake to 0 when throttle is used
                        if (newValue > 0f && brakeValue > 0f) {
                            onBrakeChange(0f)
                        }
                        onThrottleChange(newValue)
                    },
                    steps = 20,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
            
            // Brake control slider
            Column(modifier = Modifier.padding(vertical = 8.dp)) {
                Text(
                    "Brake: ${(brakeValue * 100).toInt()}%",
                    style = MaterialTheme.typography.bodyMedium
                )
                
                Slider(
                    value = brakeValue,
                    onValueChange = { newValue ->
                        // Automatically set throttle to 0 when brake is used
                        if (newValue > 0f && throttleValue > 0f) {
                            onThrottleChange(0f)
                        }
                        onBrakeChange(newValue)
                    },
                    steps = 20,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
            
            // Animation speed control
            Column(modifier = Modifier.padding(vertical = 8.dp)) {
                Text(
                    "Animation Speed: ${animationSpeed}x",
                    style = MaterialTheme.typography.bodyMedium
                )
                
                Slider(
                    value = animationSpeed,
                    onValueChange = onAnimationSpeedChange,
                    valueRange = 0.1f..2.0f,
                    steps = 19,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
            
            // Remote control note
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(top = 12.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    imageVector = Icons.Default.Sync,
                    contentDescription = null,
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.size(16.dp)
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = "Two-way remote control enabled",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.primary
                )
            }
        }
    }
    
    // Information dialog about two-way communication
    if (showTwoWayInfo) {
        AlertDialog(
            onDismissRequest = { showTwoWayInfo = false },
            title = { Text("Remote Control") },
            text = { 
                Text(
                    "These controls send commands directly to the OBD emulator. " +
                    "Changes made here will be reflected in real-time on the emulator. " +
                    "Similarly, changes made remotely on the emulator will be reflected here."
                ) 
            },
            confirmButton = {
                Button(onClick = { showTwoWayInfo = false }) {
                    Text("Got it")
                }
            }
        )
    }
}

@Preview(showBackground = true)
@Composable
fun SimulationControlPanelPreview() {
    SimulationControlPanel(
        throttleValue = 0.3f,
        onThrottleChange = {},
        brakeValue = 0.0f,
        onBrakeChange = {},
        animationSpeed = 1.0f,
        onAnimationSpeedChange = {}
    )
} 