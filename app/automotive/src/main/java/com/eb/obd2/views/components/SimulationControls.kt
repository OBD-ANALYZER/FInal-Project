package com.eb.obd2.views.components

import androidx.compose.foundation.background
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
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Speed
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.eb.obd2.R

@Composable
fun ThrottleControl(
    value: Float,
    onValueChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier.padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Throttle",
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onSurface
        )
        
        Text(
            text = "${(value * 100).toInt()}%",
            style = MaterialTheme.typography.bodyLarge.copy(
                fontWeight = FontWeight.Bold
            ),
            color = MaterialTheme.colorScheme.primary
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Vertical Slider
        Box(
            modifier = Modifier
                .height(180.dp)
                .width(50.dp),
            contentAlignment = Alignment.Center
        ) {
            // Background track
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(160.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(MaterialTheme.colorScheme.surfaceVariant)
            )
            
            // Fill color
            Box(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .height(160.dp * value)
                    .clip(RoundedCornerShape(8.dp))
                    .background(
                        brush = Brush.verticalGradient(
                            colors = listOf(
                                Color(0xFF4CAF50), // Green
                                Color(0xFFFFC107), // Yellow
                                Color(0xFFF44336)  // Red
                            ),
                            startY = 0f,
                            endY = 300f
                        )
                    )
            )
            
            // Slider control
            Slider(
                value = value,
                onValueChange = onValueChange,
                modifier = Modifier
                    .height(160.dp)
                    .width(50.dp),
                valueRange = 0f..1f,
                colors = SliderDefaults.colors(
                    thumbColor = MaterialTheme.colorScheme.primary,
                ),
                orientation = Orientation.Vertical
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Quick buttons
        Row(
            horizontalArrangement = Arrangement.SpaceEvenly,
            modifier = Modifier.fillMaxWidth()
        ) {
            IconButton(
                onClick = { 
                    // Decrease throttle by 10%
                    val newValue = (value - 0.1f).coerceIn(0f, 1f)
                    onValueChange(newValue)
                },
                modifier = Modifier.size(40.dp)
            ) {
                Icon(
                    Icons.Default.KeyboardArrowDown,
                    contentDescription = "Decrease Throttle",
                    tint = MaterialTheme.colorScheme.primary
                )
            }
            
            IconButton(
                onClick = { 
                    // Increase throttle by 10%
                    val newValue = (value + 0.1f).coerceIn(0f, 1f)
                    onValueChange(newValue)
                },
                modifier = Modifier.size(40.dp)
            ) {
                Icon(
                    Icons.Default.KeyboardArrowUp,
                    contentDescription = "Increase Throttle",
                    tint = MaterialTheme.colorScheme.primary
                )
            }
        }
    }
}

@Composable
fun BrakeControl(
    value: Float,
    onValueChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier.padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Brake",
            style = MaterialTheme.typography.titleMedium,
            color = MaterialTheme.colorScheme.onSurface
        )
        
        Text(
            text = "${(value * 100).toInt()}%",
            style = MaterialTheme.typography.bodyLarge.copy(
                fontWeight = FontWeight.Bold
            ),
            color = MaterialTheme.colorScheme.error
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Vertical Slider
        Box(
            modifier = Modifier
                .height(180.dp)
                .width(50.dp),
            contentAlignment = Alignment.Center
        ) {
            // Background track
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(160.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(MaterialTheme.colorScheme.surfaceVariant)
            )
            
            // Fill color
            Box(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .height(160.dp * value)
                    .clip(RoundedCornerShape(8.dp))
                    .background(
                        brush = Brush.verticalGradient(
                            colors = listOf(
                                Color(0xFFE57373), // Light Red
                                Color(0xFFEF5350), // Medium Red
                                Color(0xFFB71C1C)  // Dark Red
                            ),
                            startY = 0f,
                            endY = 300f
                        )
                    )
            )
            
            // Slider control
            Slider(
                value = value,
                onValueChange = onValueChange,
                modifier = Modifier
                    .height(160.dp)
                    .width(50.dp),
                valueRange = 0f..1f,
                colors = SliderDefaults.colors(
                    thumbColor = MaterialTheme.colorScheme.error,
                ),
                orientation = Orientation.Vertical
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Quick buttons
        Row(
            horizontalArrangement = Arrangement.SpaceEvenly,
            modifier = Modifier.fillMaxWidth()
        ) {
            IconButton(
                onClick = { 
                    // Decrease brake by 10%
                    val newValue = (value - 0.1f).coerceIn(0f, 1f)
                    onValueChange(newValue)
                },
                modifier = Modifier.size(40.dp)
            ) {
                Icon(
                    Icons.Default.KeyboardArrowDown,
                    contentDescription = "Decrease Brake",
                    tint = MaterialTheme.colorScheme.error
                )
            }
            
            IconButton(
                onClick = { 
                    // Increase brake by 10%
                    val newValue = (value + 0.1f).coerceIn(0f, 1f)
                    onValueChange(newValue)
                },
                modifier = Modifier.size(40.dp)
            ) {
                Icon(
                    Icons.Default.KeyboardArrowUp,
                    contentDescription = "Increase Brake",
                    tint = MaterialTheme.colorScheme.error
                )
            }
        }
    }
}

@Composable
fun AnimationSpeedControl(
    value: Float,
    onValueChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier.padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            Icon(
                Icons.Default.Speed,
                contentDescription = "Animation Speed",
                tint = MaterialTheme.colorScheme.primary,
                modifier = Modifier.size(24.dp)
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            Text(
                text = "Animation Speed",
                style = MaterialTheme.typography.titleMedium,
                color = MaterialTheme.colorScheme.onSurface
            )
        }
        
        Spacer(modifier = Modifier.height(4.dp))
        
        Text(
            text = "${value}x",
            style = MaterialTheme.typography.bodyLarge.copy(
                fontWeight = FontWeight.Bold
            ),
            color = MaterialTheme.colorScheme.primary
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Horizontal Slider
        Slider(
            value = value,
            onValueChange = onValueChange,
            modifier = Modifier.fillMaxWidth(),
            valueRange = 0.1f..2.0f,
            steps = 18, // (2.0 - 0.1) / 0.1 - 1 = 18 steps
            colors = SliderDefaults.colors(
                thumbColor = MaterialTheme.colorScheme.primary,
                activeTrackColor = MaterialTheme.colorScheme.primary,
                inactiveTrackColor = MaterialTheme.colorScheme.surfaceVariant
            )
        )
        
        // Quick preset buttons
        Row(
            horizontalArrangement = Arrangement.SpaceEvenly,
            modifier = Modifier.fillMaxWidth()
        ) {
            Button(
                onClick = { onValueChange(0.5f) },
                modifier = Modifier.weight(1f).padding(end = 4.dp)
            ) {
                Text("0.5x")
            }
            
            Button(
                onClick = { onValueChange(1.0f) },
                modifier = Modifier.weight(1f).padding(horizontal = 4.dp)
            ) {
                Text("1.0x")
            }
            
            Button(
                onClick = { onValueChange(2.0f) },
                modifier = Modifier.weight(1f).padding(start = 4.dp)
            ) {
                Text("2.0x")
            }
        }
    }
}

@Composable
fun AdvancedSimulationControls(
    throttleValue: Float,
    onThrottleChange: (Float) -> Unit,
    brakeValue: Float,
    onBrakeChange: (Float) -> Unit,
    animationSpeed: Float,
    onAnimationSpeedChange: (Float) -> Unit,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier,
        shape = RoundedCornerShape(16.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            Text(
                text = "Advanced Vehicle Controls",
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(bottom = 16.dp)
            )
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceAround
            ) {
                // Throttle control
                ThrottleControl(
                    value = throttleValue,
                    onValueChange = { onThrottleChange(it) },
                    modifier = Modifier.weight(1f)
                )
                
                // Brake control
                BrakeControl(
                    value = brakeValue,
                    onValueChange = { onBrakeChange(it) },
                    modifier = Modifier.weight(1f)
                )
            }
            
            // Animation speed control
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(top = 16.dp)
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Icon(
                        imageVector = Icons.Default.Speed,
                        contentDescription = null,
                        tint = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.size(20.dp)
                    )
                    
                    Spacer(modifier = Modifier.width(8.dp))
                    
                    Text(
                        text = "Simulation Speed: ${animationSpeed}x",
                        style = MaterialTheme.typography.bodyMedium,
                        fontWeight = FontWeight.Medium
                    )
                }
                
                Slider(
                    value = animationSpeed,
                    onValueChange = onAnimationSpeedChange,
                    valueRange = 0.1f..2.0f,
                    steps = 19,
                    modifier = Modifier.padding(top = 8.dp)
                )
            }
        }
    }
}

enum class Orientation {
    Horizontal,
    Vertical
}

@Composable
private fun Slider(
    value: Float,
    onValueChange: (Float) -> Unit,
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
    valueRange: ClosedFloatingPointRange<Float> = 0f..1f,
    steps: Int = 0,
    orientation: Orientation = Orientation.Horizontal,
    colors: androidx.compose.material3.SliderColors = SliderDefaults.colors()
) {
    when (orientation) {
        Orientation.Horizontal -> {
            Slider(
                value = value,
                onValueChange = onValueChange,
                modifier = modifier,
                enabled = enabled,
                valueRange = valueRange,
                steps = steps,
                colors = colors
            )
        }
        Orientation.Vertical -> {
            // For vertical, we invert the value because slider draws from start(left/top) to end(right/bottom)
            // but we want 0 at bottom and 1 at top
            Slider(
                value = 1f - value,
                onValueChange = { onValueChange(1f - it) },
                modifier = modifier,
                enabled = enabled,
                valueRange = valueRange.let { 1f - it.endInclusive..1f - it.start },
                steps = steps,
                colors = colors
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun ThrottleControlPreview() {
    var value by remember { mutableFloatStateOf(0.3f) }
    ThrottleControl(value = value, onValueChange = { value = it })
}

@Preview(showBackground = true)
@Composable
fun BrakeControlPreview() {
    var value by remember { mutableFloatStateOf(0.6f) }
    BrakeControl(value = value, onValueChange = { value = it })
}

@Preview(showBackground = true)
@Composable
fun AnimationSpeedControlPreview() {
    var value by remember { mutableFloatStateOf(1.0f) }
    AnimationSpeedControl(value = value, onValueChange = { value = it })
}

@Preview(showBackground = true)
@Composable
fun AdvancedSimulationControlsPreview() {
    AdvancedSimulationControls(
        throttleValue = 0.3f,
        onThrottleChange = { it },
        brakeValue = 0.0f,
        onBrakeChange = { it },
        animationSpeed = 1.0f,
        onAnimationSpeedChange = { it }
    )
} 