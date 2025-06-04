package com.eb.obd2.services

import android.content.Context
import javax.inject.Inject

class OBDConnectionFactory @Inject constructor(
    private val tcpConnection: TCPOBDConnection,
    private val context: Context
) {
    private var currentConnection: OBDConnection? = null

    fun create(type: OBDConnectionType): OBDConnection {
        return when (type) {
            OBDConnectionType.TCP -> tcpConnection.apply {
                setup(context)
            }
            OBDConnectionType.COM -> throw NotImplementedError("COM connection not implemented")
        }
    }
    
    fun getCurrentConnection(): OBDConnection? {
        return currentConnection
    }
}