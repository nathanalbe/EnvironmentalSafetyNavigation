/*
 * Copyright (c) 2023 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package io.github.deltacv.vision.internal.opmode

import com.github.serivesmejia.eocvsim.util.event.EventHandler
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue
import java.util.concurrent.ArrayBlockingQueue

class OpModeNotifier(maxNotificationsQueueSize: Int = 100) {

    private val notifications = EvictingBlockingQueue<OpModeNotification>(ArrayBlockingQueue(maxNotificationsQueueSize))
    private val exceptionQueue = EvictingBlockingQueue<Throwable>(ArrayBlockingQueue(maxNotificationsQueueSize))

    private val stateLock = Any()
    var state = OpModeState.STOPPED
        private set
        get() {
            synchronized(stateLock) {
                return field
            }
        }

    val onStateChange = EventHandler("OpModeNotifier-onStateChange")

    fun notify(notification: OpModeNotification) {
        notifications.offer(notification)
    }

    fun notify(state: OpModeState) {
        synchronized(stateLock) {
            this.state = state
        }

        onStateChange.run()
    }

    fun notify(notification: OpModeNotification, state: OpModeState) {
        notifications.offer(notification)

        synchronized(stateLock) {
            this.state = state
        }
        onStateChange.run()
    }

    fun notify(e: Throwable){
        exceptionQueue.offer(e)
    }

    fun reset() {
        notifications.clear()
        state = OpModeState.STOPPED
    }

    fun poll(): OpModeNotification {
        return notifications.poll() ?: OpModeNotification.NOTHING
    }

    fun pollException() = exceptionQueue.poll() ?: null

}