package com.eb.obd2.views.components

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.animation.expandVertically
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.shrinkVertically
import androidx.compose.foundation.background
import androidx.compose.foundation.border
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
import androidx.compose.material.icons.filled.Error
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Divider
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

enum class WarningLevel {
    CRITICAL,
    WARNING,
    INFO
}

data class VehicleWarning(
    val id: String,
    val title: String,
    val description: String,
    val level: WarningLevel,
    val timestamp: LocalDateTime = LocalDateTime.now(),
    val isNew: Boolean = true,
    val actionRequired: Boolean = false,
    val recommendedAction: String = ""
)

@Composable
fun WarningNotificationsPanel(
    warnings: List<VehicleWarning>,
    onDismissWarning: (String) -> Unit = {},
    onAcknowledgeAll: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    if (warnings.isEmpty()) {
        return
    }
    
    val criticalWarnings = warnings.filter { it.level == WarningLevel.CRITICAL }
    val normalWarnings = warnings.filter { it.level == WarningLevel.WARNING }
    val infoWarnings = warnings.filter { it.level == WarningLevel.INFO }
    
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        // Header with count
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    imageVector = getWarningIcon(
                        when {
                            criticalWarnings.isNotEmpty() -> WarningLevel.CRITICAL
                            normalWarnings.isNotEmpty() -> WarningLevel.WARNING
                            else -> WarningLevel.INFO
                        }
                    ),
                    contentDescription = "Warnings",
                    tint = getWarningColor(
                        when {
                            criticalWarnings.isNotEmpty() -> WarningLevel.CRITICAL
                            normalWarnings.isNotEmpty() -> WarningLevel.WARNING
                            else -> WarningLevel.INFO
                        }
                    ),
                    modifier = Modifier.size(28.dp)
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = "Vehicle Alerts",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
            }
            
            Row {
                if (warnings.size > 1) {
                    TextButton(onClick = onAcknowledgeAll) {
                        Text("Dismiss All")
                    }
                }
                
                Text(
                    text = "${warnings.size} ${if (warnings.size == 1) "Alert" else "Alerts"}",
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(horizontal = 8.dp, vertical = 4.dp)
                )
            }
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Critical warnings with animation
        if (criticalWarnings.isNotEmpty()) {
            AnimatedCriticalWarnings(criticalWarnings, onDismissWarning)
            
            if (normalWarnings.isNotEmpty() || infoWarnings.isNotEmpty()) {
                Spacer(modifier = Modifier.height(16.dp))
                Divider()
                Spacer(modifier = Modifier.height(16.dp))
            }
        }
        
        // Regular warnings
        if (normalWarnings.isNotEmpty() || infoWarnings.isNotEmpty()) {
            LazyColumn(
                verticalArrangement = Arrangement.spacedBy(8.dp),
                modifier = Modifier.weight(1f, fill = false)
            ) {
                items(normalWarnings + infoWarnings) { warning ->
                    WarningCard(warning, onDismissWarning)
                }
            }
        }
    }
}

@Composable
fun AnimatedCriticalWarnings(
    criticalWarnings: List<VehicleWarning>,
    onDismissWarning: (String) -> Unit
) {
    val infiniteTransition = rememberInfiniteTransition()
    val alpha by infiniteTransition.animateFloat(
        initialValue = 0.6f,
        targetValue = 1.0f,
        animationSpec = infiniteRepeatable(
            animation = tween(600, easing = LinearEasing),
            repeatMode = RepeatMode.Reverse
        )
    )
    
    Column(
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        criticalWarnings.forEach { warning ->
            Card(
                modifier = Modifier
                    .fillMaxWidth()
                    .alpha(alpha),
                colors = CardDefaults.cardColors(
                    containerColor = getWarningColor(warning.level).copy(alpha = 0.15f)
                ),
                shape = RoundedCornerShape(8.dp),
                border = androidx.compose.foundation.BorderStroke(
                    width = 2.dp,
                    color = getWarningColor(warning.level)
                )
            ) {
                Row(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(16.dp),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.Top
                ) {
                    Row(
                        verticalAlignment = Alignment.Top
                    ) {
                        Icon(
                            imageVector = getWarningIcon(warning.level),
                            contentDescription = "Critical Warning",
                            tint = getWarningColor(warning.level),
                            modifier = Modifier
                                .size(24.dp)
                                .padding(top = 2.dp)
                        )
                        
                        Spacer(modifier = Modifier.width(12.dp))
                        
                        Column {
                            Text(
                                text = warning.title,
                                style = MaterialTheme.typography.titleMedium,
                                fontWeight = FontWeight.Bold,
                                color = getWarningColor(warning.level)
                            )
                            
                            Spacer(modifier = Modifier.height(4.dp))
                            
                            Text(
                                text = warning.description,
                                style = MaterialTheme.typography.bodyMedium
                            )
                            
                            if (warning.actionRequired) {
                                Spacer(modifier = Modifier.height(8.dp))
                                
                                Row(
                                    verticalAlignment = Alignment.CenterVertically,
                                    modifier = Modifier
                                        .background(
                                            color = MaterialTheme.colorScheme.errorContainer,
                                            shape = RoundedCornerShape(4.dp)
                                        )
                                        .padding(horizontal = 8.dp, vertical = 4.dp)
                                ) {
                                    Text(
                                        text = "Action Required:",
                                        style = MaterialTheme.typography.bodySmall,
                                        fontWeight = FontWeight.Bold,
                                        color = MaterialTheme.colorScheme.onErrorContainer
                                    )
                                    
                                    Spacer(modifier = Modifier.width(4.dp))
                                    
                                    Text(
                                        text = warning.recommendedAction,
                                        style = MaterialTheme.typography.bodySmall,
                                        color = MaterialTheme.colorScheme.onErrorContainer
                                    )
                                }
                            }
                            
                            Spacer(modifier = Modifier.height(6.dp))
                            
                            Text(
                                text = formatTimestamp(warning.timestamp),
                                style = MaterialTheme.typography.bodySmall,
                                color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                            )
                        }
                    }
                    
                    TextButton(
                        onClick = { onDismissWarning(warning.id) }
                    ) {
                        Text("Dismiss")
                    }
                }
            }
        }
    }
}

@Composable
fun WarningCard(
    warning: VehicleWarning,
    onDismissWarning: (String) -> Unit
) {
    var expanded by remember { mutableStateOf(warning.isNew) }
    
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(
            containerColor = if (warning.isNew) 
                getWarningColor(warning.level).copy(alpha = 0.1f) 
            else 
                MaterialTheme.colorScheme.surface
        ),
        shape = RoundedCornerShape(8.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically,
                    modifier = Modifier.weight(1f)
                ) {
                    Icon(
                        imageVector = getWarningIcon(warning.level),
                        contentDescription = null,
                        tint = getWarningColor(warning.level),
                        modifier = Modifier.size(20.dp)
                    )
                    
                    Spacer(modifier = Modifier.width(8.dp))
                    
                    Text(
                        text = warning.title,
                        style = MaterialTheme.typography.titleSmall,
                        fontWeight = FontWeight.Bold,
                        color = if (warning.isNew) 
                            getWarningColor(warning.level) 
                        else 
                            MaterialTheme.colorScheme.onSurface
                    )
                    
                    if (warning.isNew) {
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        Box(
                            modifier = Modifier
                                .size(8.dp)
                                .background(getWarningColor(warning.level), shape = RoundedCornerShape(4.dp))
                        )
                    }
                }
                
                TextButton(
                    onClick = { expanded = !expanded }
                ) {
                    Text(if (expanded) "Hide" else "Show")
                }
            }
            
            AnimatedVisibility(
                visible = expanded,
                enter = expandVertically() + fadeIn(),
                exit = shrinkVertically() + fadeOut()
            ) {
                Column(
                    modifier = Modifier.padding(top = 8.dp)
                ) {
                    Text(
                        text = warning.description,
                        style = MaterialTheme.typography.bodyMedium
                    )
                    
                    if (warning.actionRequired) {
                        Spacer(modifier = Modifier.height(8.dp))
                        
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            modifier = Modifier
                                .fillMaxWidth()
                                .background(
                                    color = when (warning.level) {
                                        WarningLevel.CRITICAL -> MaterialTheme.colorScheme.errorContainer
                                        WarningLevel.WARNING -> MaterialTheme.colorScheme.secondaryContainer
                                        WarningLevel.INFO -> MaterialTheme.colorScheme.tertiaryContainer
                                    },
                                    shape = RoundedCornerShape(4.dp)
                                )
                                .padding(horizontal = 8.dp, vertical = 4.dp)
                        ) {
                            Text(
                                text = "Recommended action: ${warning.recommendedAction}",
                                style = MaterialTheme.typography.bodySmall,
                                color = when (warning.level) {
                                    WarningLevel.CRITICAL -> MaterialTheme.colorScheme.onErrorContainer
                                    WarningLevel.WARNING -> MaterialTheme.colorScheme.onSecondaryContainer
                                    WarningLevel.INFO -> MaterialTheme.colorScheme.onTertiaryContainer
                                }
                            )
                        }
                    }
                    
                    Spacer(modifier = Modifier.height(8.dp))
                    
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = formatTimestamp(warning.timestamp),
                            style = MaterialTheme.typography.bodySmall,
                            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                        )
                        
                        TextButton(
                            onClick = { onDismissWarning(warning.id) }
                        ) {
                            Text("Dismiss")
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun getWarningColor(level: WarningLevel): Color {
    return when (level) {
        WarningLevel.CRITICAL -> Color(0xFFF44336) // Red
        WarningLevel.WARNING -> Color(0xFFFF9800) // Orange
        WarningLevel.INFO -> Color(0xFF2196F3) // Blue
    }
}

private fun getWarningIcon(level: WarningLevel): ImageVector {
    return when (level) {
        WarningLevel.CRITICAL -> Icons.Filled.Error
        WarningLevel.WARNING -> Icons.Filled.Warning
        WarningLevel.INFO -> Icons.Filled.Info
    }
}

private fun formatTimestamp(timestamp: LocalDateTime): String {
    val formatter = DateTimeFormatter.ofPattern("HH:mm:ss")
    return timestamp.format(formatter)
}

@Preview(showBackground = true)
@Composable
fun WarningNotificationsPanelPreview() {
    MaterialTheme {
        Surface {
            val now = LocalDateTime.now()
            val warnings = listOf(
                VehicleWarning(
                    id = "1",
                    title = "Engine Overheating",
                    description = "Engine temperature critically high at 120°C. Risk of engine damage.",
                    level = WarningLevel.CRITICAL,
                    timestamp = now.minusMinutes(5),
                    actionRequired = true,
                    recommendedAction = "Pull over safely and turn off the engine"
                ),
                VehicleWarning(
                    id = "2",
                    title = "Low Oil Pressure",
                    description = "Oil pressure below recommended level. Check oil level.",
                    level = WarningLevel.WARNING,
                    timestamp = now.minusMinutes(15),
                    isNew = false
                ),
                VehicleWarning(
                    id = "3",
                    title = "Low Fuel Level",
                    description = "Fuel level at 12%. Estimated range: 45km.",
                    level = WarningLevel.INFO,
                    timestamp = now.minusMinutes(30)
                )
            )
            
            WarningNotificationsPanel(warnings)
        }
    }
} 