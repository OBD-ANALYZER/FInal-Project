package com.eb.obd2.viewmodels

import androidx.lifecycle.ViewModel
import com.eb.obd2.services.DrivingScenarioSimulator
import com.eb.obd2.services.OBDService
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class DrivingScenarioViewModel @Inject constructor(
    private val obdService: OBDService
) : ViewModel() {
    private val simulator = DrivingScenarioSimulator(obdService)

    fun getSimulator(): DrivingScenarioSimulator {
        return simulator
    }
} 