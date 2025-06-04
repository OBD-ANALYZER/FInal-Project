package com.eb.obd2.views.components

import androidx.compose.animation.animateColorAsState
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
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.eb.obd2.R

@Composable
fun GearPositionIndicator(
    gearPosition: String, // P, R, N, D
    modifier: Modifier = Modifier
) {
    val positions = listOf("P", "R", "N", "D")
    
    Row(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp),
        horizontalArrangement = Arrangement.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            Icon(
                painter = painterResource(id = R.drawable.ic_transmission),
                contentDescription = "Transmission Icon",
                tint = MaterialTheme.colorScheme.primary,
                modifier = Modifier.size(24.dp)
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            Text(
                text = "Gear Position:",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
            )
        }
        
        Spacer(modifier = Modifier.width(8.dp))
        
        Row(
            horizontalArrangement = Arrangement.spacedBy(4.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            positions.forEach { position ->
                val isSelected = position == gearPosition
                val backgroundColor by animateColorAsState(
                    targetValue = if (isSelected) {
                        when(position) {
                            "P" -> Color(0xFF2196F3) // Blue
                            "R" -> Color(0xFFF44336) // Red
                            "N" -> Color(0xFFFF9800) // Orange
                            "D" -> Color(0xFF4CAF50) // Green
                            else -> MaterialTheme.colorScheme.primary
                        }
                    } else {
                        MaterialTheme.colorScheme.surface
                    },
                    label = "gear_position_background"
                )
                
                val textColor by animateColorAsState(
                    targetValue = if (isSelected) Color.White else MaterialTheme.colorScheme.onSurface.copy(alpha = 0.6f),
                    label = "gear_position_text"
                )
                
                Box(
                    modifier = Modifier
                        .size(32.dp)
                        .clip(CircleShape)
                        .background(backgroundColor)
                        .border(
                            width = 1.dp,
                            color = if (isSelected) backgroundColor else MaterialTheme.colorScheme.outline,
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = position,
                        style = MaterialTheme.typography.bodyMedium.copy(
                            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
                        ),
                        color = textColor
                    )
                }
            }
        }
    }
}

@Composable
fun CurrentGearDisplay(
    currentGear: Int, // 1-6
    inDriveMode: Boolean = true,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "GEAR",
            style = MaterialTheme.typography.labelMedium,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
        )
        
        if (inDriveMode) {
            Text(
                text = "$currentGear",
                style = MaterialTheme.typography.headlineLarge.copy(
                    fontWeight = FontWeight.Bold,
                    fontSize = 48.sp
                ),
                color = MaterialTheme.colorScheme.primary
            )
        } else {
            Text(
                text = "-",
                style = MaterialTheme.typography.headlineLarge.copy(
                    fontWeight = FontWeight.Bold,
                    fontSize = 48.sp
                ),
                color = MaterialTheme.colorScheme.outline
            )
        }
    }
}

@Composable
fun TransmissionStatusDisplay(
    gearPosition: String, // P, R, N, D
    currentGear: Int, // 1-6
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier,
        shape = RoundedCornerShape(8.dp),
        tonalElevation = 2.dp
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            GearPositionIndicator(gearPosition = gearPosition)
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.Center,
                verticalAlignment = Alignment.CenterVertically
            ) {
                CurrentGearDisplay(
                    currentGear = currentGear,
                    inDriveMode = gearPosition == "D"
                )
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun GearPositionIndicatorPreview() {
    GearPositionIndicator(gearPosition = "D")
}

@Preview(showBackground = true)
@Composable
fun CurrentGearDisplayPreview() {
    CurrentGearDisplay(currentGear = 3)
}

@Preview(showBackground = true)
@Composable
fun TransmissionStatusDisplayPreview() {
    TransmissionStatusDisplay(gearPosition = "D", currentGear = 4)
} 