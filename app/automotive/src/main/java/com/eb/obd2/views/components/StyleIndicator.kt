package com.eb.obd2.views.components

import androidx.compose.foundation.background
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
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.eb.obd2.R
import com.eb.obd2.models.DrivingState

@Composable
fun StyleIndicator(
    aggressiveScore: Float,
    comfortLevel: Float,
    accelerationScore: Float = 1.0f,
    decelerationScore: Float = 1.0f,
    corneringScore: Float = 1.0f,
    engineEfficiencyScore: Float = 1.0f,
    drivingState: DrivingState = DrivingState.NORMAL,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        // Current driving state indicator
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(bottom = 12.dp)
        ) {
            Text(
                text = "Driving Style: ",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface
            )
            
            Text(
                text = when(drivingState) {
                    DrivingState.IDLE -> "Idle"
                    DrivingState.NORMAL -> "Normal"
                    DrivingState.AGGRESSIVE_ACCELERATION -> "Aggressive Acceleration"
                    DrivingState.AGGRESSIVE_DECELERATION -> "Aggressive Braking"
                },
                style = MaterialTheme.typography.bodyMedium.copy(fontWeight = FontWeight.Bold),
                color = when(drivingState) {
                    DrivingState.IDLE -> MaterialTheme.colorScheme.onSurface
                    DrivingState.NORMAL -> Color.Green
                    DrivingState.AGGRESSIVE_ACCELERATION, 
                    DrivingState.AGGRESSIVE_DECELERATION -> MaterialTheme.colorScheme.error
                }
            )
        }
        
        // Main indicators row
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(vertical = 4.dp)
        ) {
            // Aggressive score indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = if (aggressiveScore > 0.8) MaterialTheme.colorScheme.error else Color.Green,
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_fuel),
                        contentDescription = "Aggressiveness",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Aggr.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }

            Spacer(modifier = Modifier.width(16.dp))

            // Comfort level indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = if (comfortLevel < 0.7) MaterialTheme.colorScheme.error else Color.Green,
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_temperature),
                        contentDescription = "Comfort",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Comfort",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            Spacer(modifier = Modifier.width(16.dp))
            
            // Acceleration behavior indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = when {
                                accelerationScore < 0.5f -> MaterialTheme.colorScheme.error
                                accelerationScore < 0.8f -> Color(0xFFFFC107) // Amber
                                else -> Color.Green
                            },
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_fuel),
                        contentDescription = "Acceleration",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Accel.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            Spacer(modifier = Modifier.width(16.dp))
            
            // Deceleration/braking behavior indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = when {
                                decelerationScore < 0.5f -> MaterialTheme.colorScheme.error
                                decelerationScore < 0.8f -> Color(0xFFFFC107) // Amber
                                else -> Color.Green
                            },
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_temperature),
                        contentDescription = "Braking",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Braking",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Additional indicators row
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(vertical = 4.dp)
        ) {
            // Cornering stability indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = when {
                                corneringScore < 0.5f -> MaterialTheme.colorScheme.error
                                corneringScore < 0.8f -> Color(0xFFFFC107) // Amber
                                else -> Color.Green
                            },
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_fuel),
                        contentDescription = "Cornering",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Cornering",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            Spacer(modifier = Modifier.width(16.dp))
            
            // Engine efficiency indicator
            Column(
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Box(
                    modifier = Modifier
                        .size(40.dp)
                        .background(
                            color = when {
                                engineEfficiencyScore < 0.5f -> MaterialTheme.colorScheme.error
                                engineEfficiencyScore < 0.8f -> Color(0xFFFFC107) // Amber
                                else -> Color.Green
                            },
                            shape = CircleShape
                        ),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        painter = painterResource(R.drawable.ic_fuel),
                        contentDescription = "Engine Efficiency",
                        tint = Color.White,
                        modifier = Modifier.size(24.dp)
                    )
                }
                
                Spacer(modifier = Modifier.height(4.dp))
                
                Text(
                    text = "Efficiency",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
        }
        
        // Overall driving style feedback bar
        Spacer(modifier = Modifier.height(16.dp))
        
        Text(
            text = "Overall Driving Style",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f),
            modifier = Modifier.padding(bottom = 4.dp)
        )
        
        val overallScore = (accelerationScore + decelerationScore + corneringScore + comfortLevel) / 4
        
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(8.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(MaterialTheme.colorScheme.surfaceVariant)
        ) {
            Box(
                modifier = Modifier
                    .fillMaxWidth(overallScore)
                    .height(8.dp)
                    .clip(RoundedCornerShape(4.dp))
                    .background(
                        when {
                            overallScore < 0.5f -> MaterialTheme.colorScheme.error
                            overallScore < 0.8f -> Color(0xFFFFC107) // Amber
                            else -> Color.Green
                        }
                    )
            )
        }
    }
}

@Preview(showBackground = true)
@Composable
fun StyleIndicatorPreview() = StyleIndicator(
    aggressiveScore = 0.6f,
    comfortLevel = 0.8f,
    accelerationScore = 0.7f,
    decelerationScore = 0.9f,
    corneringScore = 0.5f,
    engineEfficiencyScore = 0.8f,
    drivingState = DrivingState.NORMAL
)