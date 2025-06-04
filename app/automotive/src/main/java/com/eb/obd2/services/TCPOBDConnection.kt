package com.eb.obd2.services

import android.content.Context
import android.content.pm.PackageManager
import android.util.Log
import com.eb.obd2.R
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout
import java.io.BufferedReader
import java.io.IOException
import java.io.InputStream
import java.io.InputStreamReader
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import javax.inject.Inject

class TCPOBDConnection @Inject constructor() : OBDConnection {
    private var socket: Socket? = null
    private var host: String = "192.168.105.103" // Default host
    //private var host: String = "192.168.1.230"
    //private var host: String = "10.104.221.84"
    private var port: Int = 3000 // Default port

    private var reader: BufferedReader? = null
    private var writer: OutputStream? = null

    companion object {
        private const val TAG = "TCPOBDConnection"
        private const val CONNECTION_TIMEOUT = 5000L // 5 seconds
        private const val READ_TIMEOUT = 1000 // 1 second
        private const val BUFFER_SIZE = 1024
    }

    override fun setup(context: Context) {
        if (context.checkSelfPermission(android.Manifest.permission.INTERNET) == PackageManager.PERMISSION_DENIED)
            throw Error("Require internet access permission.")

        // Try to get host and port from resources, fallback to defaults if not found
        try {
            host = context.getString(R.string.server_host)
            port = context.resources.getInteger(R.integer.server_port)
        } catch (e: Exception) {
            Log.w(TAG, "Using default host and port: $host:$port")
        }
    }

    override suspend fun openConnection() {
        withContext(Dispatchers.IO) {
            closeConnection() // Close any existing connection first

            withTimeout(CONNECTION_TIMEOUT) {
                try {
                    socket = Socket()
                    socket?.connect(InetSocketAddress(host, port), CONNECTION_TIMEOUT.toInt())
                    socket?.soTimeout = READ_TIMEOUT
                    socket?.keepAlive = true
                    socket?.tcpNoDelay = true
                    socket?.receiveBufferSize = BUFFER_SIZE
                    socket?.sendBufferSize = BUFFER_SIZE

                    // Initialize reader and writer
                    reader = BufferedReader(InputStreamReader(socket?.getInputStream()))
                    writer = socket?.getOutputStream()

                    Log.i(TAG, "Connected to $host:$port")
                } catch (e: Exception) {
                    Log.e(TAG, "Failed to connect to $host:$port", e)
                    closeConnection()
                    throw e
                }
            }
        }
    }

    override fun isConnected(): Boolean {
        return socket?.isConnected == true && socket?.isClosed != true
    }

    override suspend fun closeConnection() {
        withContext(Dispatchers.IO) {
            try {
                reader?.close()
                writer?.close()
                socket?.close()
            } catch (e: Exception) {
                Log.e(TAG, "Error closing connection", e)
            } finally {
                reader = null
                writer = null
                socket = null
                Log.i(TAG, "Connection closed")
            }
        }
    }

    override suspend fun sendCommand(command: String) {
        withContext(Dispatchers.IO) {
            try {
                if (!isConnected()) {
                    throw IOException("Not connected to OBD adapter")
                }

                writer?.write("${command}\r".toByteArray())
                writer?.flush()
                Log.d(TAG, "Sent command: $command")
            } catch (e: Exception) {
                Log.e(TAG, "Error sending command", e)
                closeConnection() // Close connection on error
                throw e
            }
        }
    }

    override suspend fun readResponse(): String? {
        return withContext(Dispatchers.IO) {
            try {
                if (!isConnected()) {
                    throw IOException("Not connected to OBD adapter")
                }

                val response = reader?.readLine()
                if (response == null) {
                    Log.w(TAG, "Received null response")
                    closeConnection() // Close connection on null response
                } else {
                    Log.d(TAG, "Received response: $response")
                }
                response
            } catch (e: Exception) {
                Log.e(TAG, "Error reading response", e)
                closeConnection() // Close connection on error
                null
            }
        }
    }

    override val inputStream: InputStream
        get() = socket?.getInputStream() ?: throw IOException("Not connected to OBD adapter")

    override val outputStream: OutputStream
        get() = socket?.getOutputStream() ?: throw IOException("Not connected to OBD adapter")
} 