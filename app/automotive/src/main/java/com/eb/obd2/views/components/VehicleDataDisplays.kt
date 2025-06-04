package com.eb.obd2.views.components

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Card
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.eb.obd2.R
import com.patrykandpatrick.vico.compose.cartesian.CartesianChartHost
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberAxisGuidelineComponent
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberBottomAxis
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberStartAxis
import com.patrykandpatrick.vico.compose.cartesian.layer.rememberLineCartesianLayer
import com.patrykandpatrick.vico.compose.cartesian.layer.rememberLineSpec
import com.patrykandpatrick.vico.compose.cartesian.rememberCartesianChart
import com.patrykandpatrick.vico.compose.common.component.rememberShapeComponent
import com.patrykandpatrick.vico.compose.common.component.rememberTextComponent
import com.patrykandpatrick.vico.compose.common.shape.markerCornered
import com.patrykandpatrick.vico.core.cartesian.data.CartesianChartModelProducer
import com.patrykandpatrick.vico.core.cartesian.data.lineSeries
import com.patrykandpatrick.vico.core.cartesian.marker.CartesianMarker
import com.patrykandpatrick.vico.core.cartesian.marker.DefaultCartesianMarker
import com.patrykandpatrick.vico.core.common.Dimensions
import com.patrykandpatrick.vico.core.common.shape.Corner
import com.patrykandpatrick.vico.core.common.shape.Shape
import kotlin.math.min

@Composable
fun RPMDisplay(
    rpm: Float, 
    maxRpm: Float = 6000f,
    modifier: Modifier = Modifier
) {
    val animatedRpm by animateFloatAsState(
        targetValue = rpm,
        label = "rpm_animation"
    )
    
    Column(
        modifier = modifier
            .padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "RPM",
            style = MaterialTheme.typography.labelMedium,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
        )
        
        Text(
            text = "${animatedRpm.toInt()}",
            style = MaterialTheme.typography.headlineMedium.copy(
                fontWeight = FontWeight.Bold,
                fontSize = 36.sp
            )
        )

        // RPM Gauge
        Box(
            modifier = Modifier
                .padding(top = 8.dp)
                .fillMaxWidth()
                .height(8.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(MaterialTheme.colorScheme.surfaceVariant)
        ) {
            Box(
                modifier = Modifier
                    .fillMaxWidth(min(1f, animatedRpm / maxRpm))
                    .height(8.dp)
                    .clip(RoundedCornerShape(4.dp))
                    .background(
                        brush = Brush.horizontalGradient(
                            colors = listOf(
                                Color(0xFF4CAF50), // Green
                                Color(0xFFFFC107), // Yellow
                                Color(0xFFF44336)  // Red
                            )
                        )
                    )
            )
        }
    }
}

@Composable
fun TemperatureGauge(
    temperature: Float,
    minTemp: Float = 40f,
    maxTemp: Float = 120f,
    modifier: Modifier = Modifier
) {
    val normalizedTemp = (temperature - minTemp) / (maxTemp - minTemp)
    val animatedTemp by animateFloatAsState(
        targetValue = normalizedTemp.coerceIn(0f, 1f),
        label = "temp_animation"
    )
    
    val gaugeColor = when {
        temperature < 60f -> Color(0xFF2196F3) // Blue - Cold
        temperature > 100f -> Color(0xFFF44336) // Red - Hot
        else -> Color(0xFF4CAF50) // Green - Normal
    }
    
    Column(
        modifier = modifier
            .padding(8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Engine Temperature",
            style = MaterialTheme.typography.labelMedium,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
        )
        
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            Icon(
                painter = painterResource(id = R.drawable.ic_temperature),
                contentDescription = "Temperature Icon",
                tint = gaugeColor,
                modifier = Modifier.size(28.dp)
            )
            
            Spacer(modifier = Modifier.width(8.dp))
            
            Text(
                text = String.format("%.2f°C", temperature),
                style = MaterialTheme.typography.headlineMedium.copy(
                    fontWeight = FontWeight.Bold,
                    fontSize = 32.sp
                ),
                color = gaugeColor
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        TemperatureGaugeView(animatedTemp, gaugeColor)
        
        // Temperature status text
        Text(
            text = when {
                temperature < 60f -> "Cold"
                temperature > 100f -> "Overheating"
                else -> "Normal"
            },
            style = MaterialTheme.typography.labelMedium,
            color = gaugeColor,
            fontWeight = FontWeight.Bold
        )
    }
}

@Composable
fun FuelLevelDisplay(
    fuelLevel: Float, // In percentage
    fuelConsumptionRate: Float, // Liters per 100km
    modifier: Modifier = Modifier
) {
    val animatedFuelLevel by animateFloatAsState(
        targetValue = fuelLevel.coerceIn(0f, 100f),
        label = "fuel_level_animation"
    )
    
    val fuelColor = when {
        fuelLevel < 15f -> Color(0xFFF44336) // Red - Low
        fuelLevel < 30f -> Color(0xFFFFC107) // Yellow - Medium
        else -> Color(0xFF4CAF50) // Green - Good
    }
    
    // Get MaterialTheme colors outside of Canvas scope
    val surfaceVariantColor = MaterialTheme.colorScheme.surfaceVariant
    
    Column(
        modifier = modifier
            .padding(8.dp),
        horizontalAlignment = Alignment.Start
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Icon(
                    painter = painterResource(id = R.drawable.ic_fuel),
                    contentDescription = "Fuel Icon",
                    tint = fuelColor,
                    modifier = Modifier.size(24.dp)
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = "${animatedFuelLevel.toInt()}%",
                    style = MaterialTheme.typography.bodyLarge,
                    fontWeight = FontWeight.Bold
                )
            }
            
            Text(
                text = "${String.format("%.1f", fuelConsumptionRate)} L/100km",
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Fuel Gauge
        Canvas(
            modifier = Modifier
                .fillMaxWidth()
                .height(12.dp)
        ) {
            // Background track
            drawRoundRect(
                color = surfaceVariantColor,
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(4.dp.toPx()),
                size = Size(size.width, size.height)
            )
            
            // Filled part
            drawRoundRect(
                color = fuelColor,
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(4.dp.toPx()),
                size = Size(size.width * (animatedFuelLevel / 100f), size.height)
            )
            
            // Tick marks
            val tickWidth = 1.dp.toPx()
            for (i in 1..3) {
                val x = size.width * (i / 4f)
                drawLine(
                    color = Color.White.copy(alpha = 0.5f),
                    start = Offset(x, 0f),
                    end = Offset(x, size.height),
                    strokeWidth = tickWidth,
                    cap = StrokeCap.Round
                )
            }
        }
    }
}

@Composable
fun FuelEconomyDisplay(
    instantConsumption: Float, // L/100km
    averageConsumption: Float, // L/100km
    estimatedRange: Float, // km
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .padding(8.dp)
            .fillMaxWidth()
    ) {
        Text(
            text = "Fuel Economy",
            style = MaterialTheme.typography.titleMedium,
            fontWeight = FontWeight.Bold,
            color = MaterialTheme.colorScheme.primary
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            // Instant consumption
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = "Current",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
                
                Row(
                    verticalAlignment = Alignment.Bottom
                ) {
                    Text(
                        text = String.format("%.2f", instantConsumption),
                        style = MaterialTheme.typography.titleLarge,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(modifier = Modifier.width(2.dp))
                    
                    Text(
                        text = "L/100km",
                        style = MaterialTheme.typography.labelSmall,
                        modifier = Modifier.padding(bottom = 4.dp)
                    )
                }
                
                // Efficiency indicator (lower is better)
                val efficiencyColor = when {
                    instantConsumption > 12f -> Color(0xFFF44336) // Red - Bad
                    instantConsumption > 8f -> Color(0xFFFFC107) // Yellow - Average
                    else -> Color(0xFF4CAF50) // Green - Good
                }
                
                Box(
                    modifier = Modifier
                        .padding(top = 4.dp)
                        .size(width = 60.dp, height = 8.dp)
                        .clip(RoundedCornerShape(4.dp))
                        .background(MaterialTheme.colorScheme.surfaceVariant)
                ) {
                    // Inverse scale: 15L/100km (bad) to 5L/100km (good)
                    val efficiency = (1 - ((instantConsumption - 5f) / 10f)).coerceIn(0f, 1f)
                    
                    Box(
                        modifier = Modifier
                            .fillMaxHeight()
                            .width(60.dp * efficiency)
                            .background(efficiencyColor)
                    )
                }
            }
            
            // Average consumption
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = "Average",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
                
                Row(
                    verticalAlignment = Alignment.Bottom
                ) {
                    Text(
                        text = String.format("%.2f", averageConsumption),
                        style = MaterialTheme.typography.titleLarge,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(modifier = Modifier.width(2.dp))
                    
                    Text(
                        text = "L/100km",
                        style = MaterialTheme.typography.labelSmall,
                        modifier = Modifier.padding(bottom = 4.dp)
                    )
                }
                
                // Average indicator
                val avgEfficiencyColor = when {
                    averageConsumption > 12f -> Color(0xFFF44336) // Red - Bad
                    averageConsumption > 8f -> Color(0xFFFFC107) // Yellow - Average
                    else -> Color(0xFF4CAF50) // Green - Good
                }
                
                Box(
                    modifier = Modifier
                        .padding(top = 4.dp)
                        .size(width = 60.dp, height = 8.dp)
                        .clip(RoundedCornerShape(4.dp))
                        .background(MaterialTheme.colorScheme.surfaceVariant)
                ) {
                    // Inverse scale: 15L/100km (bad) to 5L/100km (good)
                    val efficiency = (1 - ((averageConsumption - 5f) / 10f)).coerceIn(0f, 1f)
                    
                    Box(
                        modifier = Modifier
                            .fillMaxHeight()
                            .width(60.dp * efficiency)
                            .background(avgEfficiencyColor)
                    )
                }
            }
            
            // Estimated range
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = "Range",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
                
                Row(
                    verticalAlignment = Alignment.Bottom
                ) {
                    Text(
                        text = "${estimatedRange.toInt()}",
                        style = MaterialTheme.typography.titleLarge,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(modifier = Modifier.width(2.dp))
                    
                    Text(
                        text = "km",
                        style = MaterialTheme.typography.labelSmall,
                        modifier = Modifier.padding(bottom = 4.dp)
                    )
                }
                
                // Range indicator
                val rangeColor = when {
                    estimatedRange < 50f -> Color(0xFFF44336) // Red - Low
                    estimatedRange < 100f -> Color(0xFFFFC107) // Yellow - Medium
                    else -> Color(0xFF4CAF50) // Green - Good
                }
                
                Icon(
                    painter = painterResource(id = R.drawable.ic_fuel),
                    contentDescription = "Range",
                    tint = rangeColor,
                    modifier = Modifier
                        .padding(top = 4.dp)
                        .size(20.dp)
                )
            }
        }
    }
}

@Composable
fun ComprehensiveDashboard(
    rpm: Float,
    engineTemperature: Float,
    fuelLevel: Float,
    fuelConsumptionRate: Float,
    averageConsumption: Float,
    estimatedRange: Float,
    speedKmh: Float,
    throttlePosition: Float,
    brakePosition: Float,
    currentGear: Int,
    gearPosition: String,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        // Header with current speed and gear
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 16.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Speed display
            Column(
                horizontalAlignment = Alignment.Start
            ) {
                Text(
                    text = "Speed",
                    style = MaterialTheme.typography.labelSmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
                
                Text(
                    text = String.format("%.2f", speedKmh),
                    style = MaterialTheme.typography.displayLarge.copy(
                        fontWeight = FontWeight.Bold,
                        fontSize = 48.sp
                    )
                )
                
                Text(
                    text = "km/h",
                    style = MaterialTheme.typography.labelMedium,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            // Gear/transmission position display
            Column(
                horizontalAlignment = Alignment.End
            ) {
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    // Transmission position (P/R/N/D)
                    Box(
                        modifier = Modifier
                            .size(width = 40.dp, height = 32.dp)
                            .clip(RoundedCornerShape(4.dp))
                            .background(
                                if (gearPosition == "D")
                                    MaterialTheme.colorScheme.primary
                                else
                                    MaterialTheme.colorScheme.surfaceVariant
                            ),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = gearPosition,
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.Bold,
                            color = if (gearPosition == "D")
                                MaterialTheme.colorScheme.onPrimary
                            else
                                MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                    
                    // Current gear (1-6)
                    if (gearPosition == "D" && currentGear > 0) {
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        Box(
                            modifier = Modifier
                                .size(32.dp)
                                .clip(RoundedCornerShape(16.dp))
                                .background(MaterialTheme.colorScheme.primaryContainer),
                            contentAlignment = Alignment.Center
                        ) {
                            Text(
                                text = "$currentGear",
                                style = MaterialTheme.typography.titleMedium,
                                fontWeight = FontWeight.Bold,
                                color = MaterialTheme.colorScheme.onPrimaryContainer
                            )
                        }
                    }
                }
                
                Spacer(modifier = Modifier.height(8.dp))
                
                // Throttle/brake indicators
                Row(
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    // Brake indicator
                    if (brakePosition > 0.05f) {
                        Text(
                            text = "BRAKE",
                            style = MaterialTheme.typography.labelSmall,
                            color = Color(0xFFF44336),
                            fontWeight = FontWeight.Bold,
                            modifier = Modifier.padding(end = 8.dp)
                        )
                    }
                    
                    // Throttle indicator
                    Box(
                        modifier = Modifier
                            .width(100.dp)
                            .height(6.dp)
                            .clip(RoundedCornerShape(3.dp))
                            .background(MaterialTheme.colorScheme.surfaceVariant)
                    ) {
                        Box(
                            modifier = Modifier
                                .fillMaxHeight()
                                .width(100.dp * throttlePosition)
                                .background(
                                    brush = Brush.horizontalGradient(
                                        colors = listOf(
                                            Color(0xFF4CAF50), // Green
                                            Color(0xFFFFEB3B), // Yellow
                                            Color(0xFFF44336)  // Red
                                        )
                                    )
                                )
                        )
                    }
                }
            }
        }
        
        // Main gauges: RPM and Temperature
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 8.dp),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            RPMDisplay(
                rpm = rpm,
                modifier = Modifier.weight(1f)
            )
            
            TemperatureGauge(
                temperature = engineTemperature,
                modifier = Modifier.weight(1f)
            )
        }
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Fuel displays
        FuelLevelDisplay(
            fuelLevel = fuelLevel,
            fuelConsumptionRate = fuelConsumptionRate
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Fuel economy metrics
        FuelEconomyDisplay(
            instantConsumption = fuelConsumptionRate,
            averageConsumption = averageConsumption,
            estimatedRange = estimatedRange
        )
    }
}

@Preview(showBackground = true)
@Composable
fun EnhancedTemperatureGaugePreview() {
    TemperatureGauge(temperature = 85f)
}

@Preview(showBackground = true)
@Composable
fun FuelEconomyDisplayPreview() {
    FuelEconomyDisplay(
        instantConsumption = 7.8f,
        averageConsumption = 8.5f,
        estimatedRange = 420f
    )
}

@Preview(showBackground = true)
@Composable
fun ComprehensiveDashboardPreview() {
    ComprehensiveDashboard(
        rpm = 2500f,
        engineTemperature = 85f,
        fuelLevel = 65f,
        fuelConsumptionRate = 7.8f,
        averageConsumption = 8.5f,
        estimatedRange = 420f,
        speedKmh = 75f,
        throttlePosition = 0.4f,
        brakePosition = 0f,
        currentGear = 4,
        gearPosition = "D"
    )
}

/**
 * Displays a chart of historical data for a specific vehicle parameter
 */
@Composable
fun HistoricalDataChart(
    title: String,
    subtitle: String,
    unitLabel: String,
    modelProducer: CartesianChartModelProducer,
    chartColor: Color = MaterialTheme.colorScheme.primary,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column {
                Text(
                    text = title,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                
                Text(
                    text = subtitle,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
                )
            }
            
            Text(
                text = unitLabel,
                style = MaterialTheme.typography.labelMedium,
                color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
            )
        }
        
        Spacer(modifier = Modifier.height(8.dp))
        
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .height(180.dp)
        ) {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(8.dp)
            ) {
                // Chart content
                CartesianChartHost(
                    chart = rememberCartesianChart(
                        rememberLineCartesianLayer(),
                        startAxis = rememberStartAxis(),
                        bottomAxis = rememberBottomAxis(guideline = null),
                    ),
                    modelProducer = modelProducer,
                    marker = createChartMarker(),
                    modifier = Modifier.fillMaxSize()
                )
            }
        }
    }
}

/**
 * Displays detailed driving analysis with scores and metrics
 */
@Composable
fun DrivingAnalysisPanel(
    overallScore: Float,
    efficiencyScore: Float,
    smoothnessScore: Float,
    aggressivenessScore: Float,
    comfortScore: Float,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        Text(
            text = "Driving Analysis",
            style = MaterialTheme.typography.titleLarge,
            fontWeight = FontWeight.Bold
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Overall score display
        Column(
            modifier = Modifier.fillMaxWidth(),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = "Overall Score",
                style = MaterialTheme.typography.titleMedium
            )
            
            Text(
                text = "${overallScore.toInt()}",
                style = MaterialTheme.typography.displayLarge.copy(
                    fontWeight = FontWeight.Bold,
                    fontSize = 56.sp,
                    color = when {
                        overallScore >= 80 -> Color(0xFF4CAF50) // Green - Excellent
                        overallScore >= 60 -> Color(0xFF8BC34A) // Light Green - Good
                        overallScore >= 40 -> Color(0xFFFFC107) // Yellow - Average
                        overallScore >= 20 -> Color(0xFFFF9800) // Orange - Poor
                        else -> Color(0xFFF44336) // Red - Bad
                    }
                )
            )
            
            Text(
                text = when {
                    overallScore >= 80 -> "Excellent driving!"
                    overallScore >= 60 -> "Good driving"
                    overallScore >= 40 -> "Average driving"
                    overallScore >= 20 -> "Needs improvement"
                    else -> "Poor driving habits"
                },
                style = MaterialTheme.typography.bodyMedium
            )
        }
        
        Spacer(modifier = Modifier.height(24.dp))
        
        // Individual metrics
        Column(
            modifier = Modifier.fillMaxWidth()
        ) {
            Text(
                text = "Detailed Metrics",
                style = MaterialTheme.typography.titleSmall,
                modifier = Modifier.padding(bottom = 8.dp)
            )
            
            // Fuel Efficiency Score
            MetricScoreBar(
                label = "Fuel Efficiency",
                score = efficiencyScore,
                iconResId = R.drawable.ic_fuel
            )
            
            Spacer(modifier = Modifier.height(12.dp))
            
            // Smoothness Score
            MetricScoreBar(
                label = "Driving Smoothness",
                score = smoothnessScore,
                iconResId = R.drawable.ic_temperature
            )
            
            Spacer(modifier = Modifier.height(12.dp))
            
            // Aggressiveness Score (inverted - lower is better)
            MetricScoreBar(
                label = "Non-Aggressive Driving",
                score = aggressivenessScore,
                iconResId = R.drawable.ic_fuel
            )
            
            Spacer(modifier = Modifier.height(12.dp))
            
            // Comfort Score
            MetricScoreBar(
                label = "Passenger Comfort",
                score = comfortScore,
                iconResId = R.drawable.ic_temperature
            )
        }
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Improvement tips based on lowest score
        val (lowestMetric, lowestScore) = listOf(
            "Fuel Efficiency" to efficiencyScore,
            "Driving Smoothness" to smoothnessScore,
            "Non-Aggressive Driving" to aggressivenessScore,
            "Passenger Comfort" to comfortScore
        ).minByOrNull { it.second } ?: ("" to 1.0f)
        
        if (lowestScore < 0.6f) {
            Card(
                modifier = Modifier.fillMaxWidth(),
            ) {
                Column(
                    modifier = Modifier.padding(16.dp)
                ) {
                    Row(
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Icon(
                            painter = painterResource(id = R.drawable.ic_temperature),
                            contentDescription = "Tip",
                            tint = MaterialTheme.colorScheme.primary,
                            modifier = Modifier.size(24.dp)
                        )
                        
                        Spacer(modifier = Modifier.width(8.dp))
                        
                        Text(
                            text = "Improvement Tip",
                            style = MaterialTheme.typography.titleSmall,
                            fontWeight = FontWeight.Bold
                        )
                    }
                    
                    Spacer(modifier = Modifier.height(8.dp))
                    
                    Text(
                        text = when (lowestMetric) {
                            "Fuel Efficiency" -> "Try to maintain steady speed and avoid heavy acceleration to improve fuel economy."
                            "Driving Smoothness" -> "Focus on gradual acceleration and braking for smoother driving."
                            "Non-Aggressive Driving" -> "Reduce rapid acceleration and hard braking for safer, less aggressive driving."
                            "Passenger Comfort" -> "Smoother transitions between acceleration and braking will improve passenger comfort."
                            else -> "Focus on improving your overall driving habits."
                        },
                        style = MaterialTheme.typography.bodyMedium
                    )
                }
            }
        }
    }
}

/**
 * A score bar for an individual metric with icon and label
 */
@Composable
private fun MetricScoreBar(
    label: String,
    score: Float, // 0.0-1.0
    iconResId: Int,
    modifier: Modifier = Modifier
) {
    val normalizedScore = score.coerceIn(0f, 1f)
    val scoreColor = when {
        normalizedScore >= 0.8f -> Color(0xFF4CAF50) // Green - Excellent
        normalizedScore >= 0.6f -> Color(0xFF8BC34A) // Light Green - Good
        normalizedScore >= 0.4f -> Color(0xFFFFC107) // Yellow - Average
        normalizedScore >= 0.2f -> Color(0xFFFF9800) // Orange - Poor
        else -> Color(0xFFF44336) // Red - Bad
    }
    
    Column(
        modifier = modifier.fillMaxWidth()
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    painter = painterResource(id = iconResId),
                    contentDescription = label,
                    tint = scoreColor,
                    modifier = Modifier.size(20.dp)
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = label,
                    style = MaterialTheme.typography.bodyMedium
                )
            }
            
            Text(
                text = "${(normalizedScore * 100).toInt()}%",
                style = MaterialTheme.typography.bodyMedium,
                color = scoreColor,
                fontWeight = FontWeight.Bold
            )
        }
        
        Spacer(modifier = Modifier.height(4.dp))
        
        // Score bar
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(8.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(MaterialTheme.colorScheme.surfaceVariant)
        ) {
            Box(
                modifier = Modifier
                    .fillMaxWidth(normalizedScore)
                    .height(8.dp)
                    .clip(RoundedCornerShape(4.dp))
                    .background(scoreColor)
            )
        }
    }
}

/**
 * A comprehensive historical data display with tabs for different metrics
 */
@Composable
fun HistoricalDataDisplay(
    temperatureModelProducer: CartesianChartModelProducer,
    fuelConsumptionModelProducer: CartesianChartModelProducer,
    fuelLevelModelProducer: CartesianChartModelProducer,
    speedModelProducer: CartesianChartModelProducer,
    modifier: Modifier = Modifier
) {
    var selectedTabIndex by remember { mutableStateOf(0) }
    
    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        Text(
            text = "Historical Data",
            style = MaterialTheme.typography.titleLarge,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(bottom = 16.dp)
        )
        
        // Tab row
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 16.dp),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            HistoryTab(
                text = "Temperature",
                isSelected = selectedTabIndex == 0,
                onClick = { selectedTabIndex = 0 }
            )
            
            HistoryTab(
                text = "Fuel Consumption",
                isSelected = selectedTabIndex == 1,
                onClick = { selectedTabIndex = 1 }
            )
            
            HistoryTab(
                text = "Fuel Level",
                isSelected = selectedTabIndex == 2,
                onClick = { selectedTabIndex = 2 }
            )
            
            HistoryTab(
                text = "Speed",
                isSelected = selectedTabIndex == 3,
                onClick = { selectedTabIndex = 3 }
            )
        }
        
        // Content based on selected tab
        when (selectedTabIndex) {
            0 -> HistoricalDataChart(
                title = "Engine Temperature",
                subtitle = "Last 100 readings",
                unitLabel = "°C",
                modelProducer = temperatureModelProducer,
                chartColor = Color(0xFF2196F3)
            )
            1 -> HistoricalDataChart(
                title = "Fuel Consumption",
                subtitle = "Last 100 readings",
                unitLabel = "L/100km",
                modelProducer = fuelConsumptionModelProducer,
                chartColor = Color(0xFF4CAF50)
            )
            2 -> HistoricalDataChart(
                title = "Fuel Level",
                subtitle = "Last 100 readings",
                unitLabel = "%",
                modelProducer = fuelLevelModelProducer,
                chartColor = Color(0xFFFFC107)
            )
            3 -> HistoricalDataChart(
                title = "Vehicle Speed",
                subtitle = "Last 100 readings",
                unitLabel = "km/h",
                modelProducer = speedModelProducer,
                chartColor = Color(0xFFF44336)
            )
        }
    }
}

/**
 * Tab for the historical data display
 */
@Composable
private fun HistoryTab(
    text: String,
    isSelected: Boolean,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(16.dp))
            .background(
                if (isSelected) MaterialTheme.colorScheme.primaryContainer
                else MaterialTheme.colorScheme.surface
            )
            .clickable(onClick = onClick)
            .padding(horizontal = 12.dp, vertical = 8.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            style = MaterialTheme.typography.labelMedium,
            color = if (isSelected) 
                MaterialTheme.colorScheme.onPrimaryContainer
            else 
                MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f),
            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal
        )
    }
}

@Preview(showBackground = true)
@Composable
fun DrivingAnalysisPanelPreview() {
    DrivingAnalysisPanel(
        overallScore = 78f,
        efficiencyScore = 0.8f,
        smoothnessScore = 0.7f,
        aggressivenessScore = 0.9f,
        comfortScore = 0.6f
    )
}

@Preview(showBackground = true)
@Composable
fun HistoricalDataChartPreview() {
    val mockProducer = CartesianChartModelProducer.build()
    mockProducer.tryRunTransaction {
        lineSeries {
            // Mock data points
            series(listOf(70f, 72f, 75f, 80f, 85f, 82f, 78f, 76f, 74f))
        }
    }
    
    HistoricalDataChart(
        title = "Engine Temperature",
        subtitle = "Last 100 readings",
        unitLabel = "°C",
        modelProducer = mockProducer,
        chartColor = Color(0xFF2196F3)
    )
}

@Preview(showBackground = true)
@Composable
fun HistoricalDataDisplayPreview() {
    val mockProducer = CartesianChartModelProducer.build()
    mockProducer.tryRunTransaction {
        lineSeries {
            // Mock data points
            series(listOf(70f, 72f, 75f, 80f, 85f, 82f, 78f, 76f, 74f))
        }
    }
    
    HistoricalDataDisplay(
        temperatureModelProducer = mockProducer,
        fuelConsumptionModelProducer = mockProducer,
        fuelLevelModelProducer = mockProducer,
        speedModelProducer = mockProducer
    )
}

@Composable
fun TemperatureHistoryGraph(
    temperatureModelProducer: CartesianChartModelProducer,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .height(150.dp)
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        CartesianChartHost(
            chart = rememberCartesianChart(
                rememberLineCartesianLayer(),
                startAxis = rememberStartAxis(),
                bottomAxis = rememberBottomAxis(guideline = null),
            ),
            modelProducer = temperatureModelProducer,
            marker = createChartMarker(),
        )
    }
}

@Composable
fun SpeedHistoryGraph(
    speedModelProducer: CartesianChartModelProducer,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .height(150.dp)
            .padding(8.dp)
    ) {
        CartesianChartHost(
            chart = rememberCartesianChart(
                rememberLineCartesianLayer(),
                startAxis = rememberStartAxis(),
                bottomAxis = rememberBottomAxis(guideline = null),
            ),
            modelProducer = speedModelProducer,
            marker = createChartMarker(),
        )
    }
} 