package com.eb.obd2.views.components

import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.patrykandpatrick.vico.compose.cartesian.CartesianChartHost
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberAxisGuidelineComponent
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberBottomAxis
import com.patrykandpatrick.vico.compose.cartesian.axis.rememberStartAxis
import com.patrykandpatrick.vico.compose.cartesian.layer.rememberLineCartesianLayer
import com.patrykandpatrick.vico.compose.cartesian.rememberCartesianChart
import com.patrykandpatrick.vico.compose.common.component.rememberLayeredComponent
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
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import kotlin.random.Random

// Create a simple marker
@Composable
fun createSimpleMarker(): CartesianMarker {
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

@Composable
fun ScoreGraph(modelProducer: CartesianChartModelProducer) {
    CartesianChartHost(
        chart = rememberCartesianChart(
            rememberLineCartesianLayer(),
            startAxis = rememberStartAxis(),
            bottomAxis = rememberBottomAxis(guideline = null),
        ),
        modelProducer = modelProducer,
        marker = createSimpleMarker()
    )
}

@Preview(showBackground = true)
@Composable
fun ScoreGraphPreview() {
    val modelProducer = remember { CartesianChartModelProducer.build() }

    LaunchedEffect(Unit) {
        withContext(Dispatchers.Default) {
            while (true) {
                modelProducer.tryRunTransaction {
                    val x = (1..50).toList();
                    lineSeries { series(x = x, y = x.map { Random.nextFloat() * 15 }) }
                }
                delay(2000L)
            }
        }
    }

    ScoreGraph(modelProducer)
}