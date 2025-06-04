package com.eb.obd2.views.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import com.patrykandpatrick.vico.compose.common.component.rememberShapeComponent
import com.patrykandpatrick.vico.compose.common.component.rememberTextComponent
import com.patrykandpatrick.vico.compose.common.shape.markerCornered
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberAxisGuidelineComponent
import com.patrykandpatrick.vico.core.cartesian.marker.CartesianMarker
import com.patrykandpatrick.vico.core.cartesian.marker.DefaultCartesianMarker
import com.patrykandpatrick.vico.core.common.Dimensions
import com.patrykandpatrick.vico.core.common.shape.Corner
import com.patrykandpatrick.vico.core.common.shape.Shape
import kotlin.math.cos
import kotlin.math.sin

@Composable
fun TemperatureGaugeView(animatedTemp: Float, gaugeColor: Color) {
    // Define colors outside the Canvas scope
    val surfaceVariantColor = MaterialTheme.colorScheme.surfaceVariant
    val colors = listOf(
        Color(0xFF2196F3), // Cold - Blue
        Color(0xFF4CAF50), // Normal - Green
        Color(0xFFF44336)  // Hot - Red
    )
    
    Canvas(
        modifier = Modifier
            .size(140.dp, 70.dp)
            .padding(8.dp)
    ) {
        val centerX = size.width / 2
        val centerY = size.height
        val radius = size.height - 4.dp.toPx()
        val strokeWidth = 12.dp.toPx()
        
        // Background arc
        drawArc(
            color = surfaceVariantColor,
            startAngle = 180f,
            sweepAngle = 180f,
            useCenter = false,
            topLeft = Offset(centerX - radius, centerY - radius),
            size = Size(radius * 2, radius * 2),
            style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
        )
        
        // Temperature arc fill
        val tempGradient = Brush.sweepGradient(
            colorStops = arrayOf(
                0.0f to colors[0],
                0.5f to colors[1],
                1.0f to colors[2]
            ),
            center = Offset(centerX, centerY)
        )
        
        // Temperature arc fill
        drawArc(
            brush = tempGradient,
            startAngle = 180f,
            sweepAngle = 180f * animatedTemp,
            useCenter = false,
            topLeft = Offset(centerX - radius, centerY - radius),
            size = Size(radius * 2, radius * 2),
            style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
        )
        
        // Temperature tick marks
        for (i in 0..4) {
            val angle = 180f + (180f * i / 4f)
            val radians = Math.toRadians(angle.toDouble())
            val outerX = centerX + (radius + 4.dp.toPx()) * cos(radians).toFloat()
            val outerY = centerY + (radius + 4.dp.toPx()) * sin(radians).toFloat()
            val innerX = centerX + (radius - 8.dp.toPx()) * cos(radians).toFloat()
            val innerY = centerY + (radius - 8.dp.toPx()) * sin(radians).toFloat()
            
            drawLine(
                color = Color.White.copy(alpha = 0.7f),
                start = Offset(innerX, innerY),
                end = Offset(outerX, outerY),
                strokeWidth = 2.dp.toPx(),
                cap = StrokeCap.Round
            )
        }
        
        // Temperature indicator needle
        val needleAngle = 180f + (180f * animatedTemp)
        val needleRadians = Math.toRadians(needleAngle.toDouble())
        val needleX = centerX + (radius - 16.dp.toPx()) * cos(needleRadians).toFloat()
        val needleY = centerY + (radius - 16.dp.toPx()) * sin(needleRadians).toFloat()
        
        drawLine(
            color = gaugeColor,
            start = Offset(centerX, centerY),
            end = Offset(needleX, needleY),
            strokeWidth = 3.dp.toPx(),
            cap = StrokeCap.Round
        )
        
        // Needle center point
        drawCircle(
            color = gaugeColor,
            radius = 5.dp.toPx(),
            center = Offset(centerX, centerY)
        )
    }
}

@Composable
fun createChartMarker(): CartesianMarker {
    val labelBackground = rememberShapeComponent(
        shape = Shape.markerCornered(Corner.FullyRounded),
        color = MaterialTheme.colorScheme.surface
    )
    
    val label = rememberTextComponent(
        color = MaterialTheme.colorScheme.onSurface,
        background = labelBackground,
        padding = Dimensions(8.dp.value, 4.dp.value, 8.dp.value, 4.dp.value)
    )
    
    val guideline = rememberAxisGuidelineComponent()
    
    return remember(label, guideline) {
        DefaultCartesianMarker(
            label = label,
            guideline = guideline
        )
    }
} 