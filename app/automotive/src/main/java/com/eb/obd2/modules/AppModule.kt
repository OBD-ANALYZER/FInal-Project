package com.eb.obd2.modules

import android.content.Context
import com.eb.obd2.repositories.ConfigurationRepository
import com.eb.obd2.repositories.PluginRepository
import com.eb.obd2.repositories.RecordRepository
import com.eb.obd2.repositories.source.persistent.RecordDao
import com.eb.obd2.repositories.source.persistent.SpeedDao
import com.eb.obd2.services.OBDService
import com.eb.obd2.services.PluginService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object AppModule {
    
    @Provides
    @Singleton
    fun provideRecordRepository(
        recordDao: RecordDao,
        speedDao: SpeedDao
    ): RecordRepository {
        return RecordRepository(recordDao, speedDao)
    }
    
    @Provides
    @Singleton
    fun provideConfigurationRepository(
        @ApplicationContext context: Context
    ): ConfigurationRepository {
        return ConfigurationRepository(context)
    }
    
    @Provides
    @Singleton
    fun providePluginRepository(
        @ApplicationContext context: Context
    ): PluginRepository {
        return PluginRepository(context)
    }
    
    @Provides
    @Singleton
    fun providePluginService(
        @ApplicationContext context: Context,
        pluginRepository: PluginRepository
    ): PluginService {
        return PluginService(context, pluginRepository)
    }
} 