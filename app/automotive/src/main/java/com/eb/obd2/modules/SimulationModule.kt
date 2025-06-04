package com.eb.obd2.modules

import com.eb.obd2.services.DrivingScenarioSimulator
import com.eb.obd2.services.OBDService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object SimulationModule {
    
    @Provides
    @Singleton
    fun provideDrivingScenarioSimulator(
        obdService: OBDService
    ): DrivingScenarioSimulator {
        return DrivingScenarioSimulator(obdService)
    }
} 