package com.eb.obd2.views.components

import androidx.compose.foundation.border
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
import androidx.compose.material.icons.filled.DirectionsCar
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Divider
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
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
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.eb.obd2.services.DrivingScenarioSimulator
import com.eb.obd2.services.DrivingScenarioType

@Composable
fun ScenarioSelectionPanel(
    scenarioSimulator: DrivingScenarioSimulator,
    modifier: Modifier = Modifier
) {
    val isRunning by scenarioSimulator.isRunning.collectAsState()
    val activeScenario by scenarioSimulator.activeScenario.collectAsState()
    val timeRemaining by scenarioSimulator.timeRemaining.collectAsState()
    val statusMessage by scenarioSimulator.statusMessage.collectAsState()
    
    val scenarios = scenarioSimulator.getAvailableScenarios()
    
    Card(
        modifier = modifier,
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            // Header
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Driving Scenarios",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
                
                if (isRunning) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        // Progress indicator
                        CircularProgressIndicator(
                            modifier = Modifier.size(24.dp),
                            color = MaterialTheme.colorScheme.primary,
                            strokeWidth = 2.dp
                        )
                        
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        // Stop button
                        IconButton(
                            onClick = { scenarioSimulator.stopScenario() }
                        ) {
                            Icon(
                                imageVector = Icons.Default.Stop,
                                contentDescription = "Stop Scenario",
                                tint = MaterialTheme.colorScheme.error
                            )
                        }
                    }
                }
            }
            
            Spacer(modifier = Modifier.height(8.dp))
            
            // Active scenario info
            if (isRunning && activeScenario != null) {
                ActiveScenarioStatus(
                    scenarioName = activeScenario.toString().replace("_", " "),
                    timeRemaining = timeRemaining,
                    onStop = { scenarioSimulator.stopScenario() }
                )
                
                Spacer(modifier = Modifier.height(16.dp))
                Divider()
                Spacer(modifier = Modifier.height(16.dp))
            }
            
            // Status message
            statusMessage?.let { message ->
                Text(
                    text = message,
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.secondary
                )
                
                Spacer(modifier = Modifier.height(16.dp))
            }
            
            // Scenario list
            LazyColumn(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                items(scenarios) { (id, description) ->
                    ScenarioItem(
                        id = id,
                        description = description,
                        isActive = activeScenario?.name?.lowercase()?.contains(id.lowercase()) == true,
                        isRunning = isRunning,
                        onStart = { scenarioSimulator.startScenarioByName(id) }
                    )
                }
            }
        }
    }
}

@Composable
fun ActiveScenarioStatus(
    scenarioName: String,
    timeRemaining: Long,
    onStop: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(
                width = 1.dp,
                color = MaterialTheme.colorScheme.primary.copy(alpha = 0.2f),
                shape = RoundedCornerShape(8.dp)
            )
            .padding(12.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "Running: $scenarioName",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold,
                color = MaterialTheme.colorScheme.primary
            )
            
            TextButton(onClick = onStop) {
                Text("Stop")
            }
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Progress indicator
        val progress = if (timeRemaining <= 0) 0f else timeRemaining / 120000f
        LinearProgressIndicator(
            progress = { 1f - progress },
            modifier = Modifier.fillMaxWidth(),
            color = MaterialTheme.colorScheme.primary,
            trackColor = MaterialTheme.colorScheme.surfaceVariant
        )
        
        Spacer(modifier = Modifier.height(4.dp))
        
        // Remaining time
        val remainingSecs = (timeRemaining / 1000).toInt()
        Text(
            text = "Time remaining: ${remainingSecs / 60}:${String.format("%02d", remainingSecs % 60)}",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
    }
}

@Composable
fun ScenarioItem(
    id: String,
    description: String,
    isActive: Boolean,
    isRunning: Boolean,
    onStart: () -> Unit
) {
    var showMenu by remember { mutableStateOf(false) }
    
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .clickable(enabled = !isRunning) {
                if (!isRunning) onStart()
            },
        colors = CardDefaults.cardColors(
            containerColor = if (isActive) 
                MaterialTheme.colorScheme.primaryContainer 
            else 
                MaterialTheme.colorScheme.surface
        )
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(12.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Scenario icon
            Icon(
                imageVector = Icons.Default.DirectionsCar,
                contentDescription = null,
                tint = if (isActive) 
                    MaterialTheme.colorScheme.primary 
                else 
                    MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f),
                modifier = Modifier.size(28.dp)
            )
            
            Spacer(modifier = Modifier.width(12.dp))
            
            // Scenario details
            Column(
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = id.replaceFirstChar { it.uppercase() },
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = if (isActive) FontWeight.Bold else FontWeight.Normal,
                    color = if (isActive) 
                        MaterialTheme.colorScheme.onPrimaryContainer 
                    else 
                        MaterialTheme.colorScheme.onSurface
                )
                
                Text(
                    text = description,
                    style = MaterialTheme.typography.bodySmall,
                    color = if (isActive) 
                        MaterialTheme.colorScheme.onPrimaryContainer.copy(alpha = 0.7f) 
                    else 
                        MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f),
                    maxLines = 2,
                    overflow = TextOverflow.Ellipsis
                )
            }
            
            // Start button
            if (!isActive && !isRunning) {
                IconButton(onClick = onStart) {
                    Icon(
                        imageVector = Icons.Default.PlayArrow,
                        contentDescription = "Start Scenario",
                        tint = MaterialTheme.colorScheme.primary
                    )
                }
            }
            
            // More options
            Box {
                IconButton(onClick = { showMenu = true }) {
                    Icon(
                        imageVector = Icons.Default.MoreVert,
                        contentDescription = "More Options",
                        tint = if (isActive) 
                            MaterialTheme.colorScheme.onPrimaryContainer 
                        else 
                            MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                    )
                }
                
                DropdownMenu(
                    expanded = showMenu,
                    onDismissRequest = { showMenu = false }
                ) {
                    DropdownMenuItem(
                        text = { Text("Start") },
                        onClick = {
                            onStart()
                            showMenu = false
                        },
                        enabled = !isRunning
                    )
                }
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun ScenarioItemPreview() {
    MaterialTheme {
        Surface {
            Column(modifier = Modifier.padding(16.dp)) {
                ScenarioItem(
                    id = "city",
                    description = "City Driving - Start-stop traffic, moderate speeds",
                    isActive = false,
                    isRunning = false,
                    onStart = {}
                )
                
                Spacer(modifier = Modifier.height(16.dp))
                
                ScenarioItem(
                    id = "highway",
                    description = "Highway Driving - Higher speeds, steady RPM",
                    isActive = true,
                    isRunning = true,
                    onStart = {}
                )
            }
        }
    }
} 