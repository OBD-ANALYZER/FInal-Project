package com.eb.obd2.services

import android.content.Context
import java.io.InputStream
import java.io.OutputStream

interface OBDConnection {
    /** Setup the connection (optional) */
    fun setup(context: Context) {}

    /** Start the connection */
    suspend fun openConnection()

    /** Close the connection */
    suspend fun closeConnection()

    /** Send a command */
    suspend fun sendCommand(command: String)

    /** Read response */
    suspend fun readResponse(): String?

    /** Check if the connection is established */
    fun isConnected(): Boolean

    /** Get input stream */
    val inputStream: InputStream

    /** Get output stream */
    val outputStream: OutputStream
}