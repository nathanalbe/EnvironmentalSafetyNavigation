/*
 * Copyright (c) 2021 Sebastian Erives
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
package com.github.serivesmejia.eocvsim.pipeline.util;

import com.github.serivesmejia.eocvsim.pipeline.PipelineData
import com.github.serivesmejia.eocvsim.pipeline.PipelineManager
import com.github.serivesmejia.eocvsim.util.event.EventHandler
import com.github.serivesmejia.eocvsim.util.StrUtil
import com.github.serivesmejia.eocvsim.util.Log

class PipelineExceptionTracker(private val pipelineManager: PipelineManager) {

    companion object {
        private const val TAG = "PipelineExceptionTracker"

        const val millisExceptionExpire = 5000L
    }

    var currentPipeline: PipelineData? = null
        private set

    val exceptionsThrown = mutableMapOf<Throwable, PipelineException>()

    val onPipelineException      = EventHandler("OnPipelineException")
    val onNewPipelineException   = EventHandler("OnNewPipelineException")
    val onPipelineExceptionClear = EventHandler("OnPipelineExceptionClear")

    val onUpdate = EventHandler("OnPipelineExceptionTrackerUpdate")

    fun update(data: PipelineData, ex: Throwable?) {
        if(currentPipeline != data) {
            exceptionsThrown.clear()
            currentPipeline = data
        }

        val exStr = if(ex != null) StrUtil.fromException(ex) else ""

        if(ex != null) {
            onPipelineException.run()

            val exception = exceptionsThrown.values.stream().filter {
                it.stacktrace == exStr
            }.findFirst()

            if(!exception.isPresent) {
                Log.blank()
                Log.warn(
                    TAG, "Uncaught exception thrown while processing pipeline ${data.clazz.simpleName}",
                    ex
                )

                Log.warn(TAG, "Note that to avoid spam, continuously equal thrown exceptions are only logged once.")
                Log.warn(TAG, "It will be reported once the pipeline stops throwing the exception after $millisExceptionExpire ms")
                Log.blank()

                exceptionsThrown[ex] = PipelineException(
                    0, exStr, System.currentTimeMillis()
                )

                onNewPipelineException.run()
            }
        }

        for((e, d) in exceptionsThrown.entries.toTypedArray()) {
            if(ex != null && d.stacktrace == exStr) {
                d.count++
                d.millisThrown = System.currentTimeMillis()
            }

            val timeElapsed = System.currentTimeMillis() - d.millisThrown
            if(timeElapsed >= millisExceptionExpire) {
                exceptionsThrown.remove(e)
                Log.info(
                    TAG,
                    "Pipeline ${currentPipeline!!.clazz.simpleName} stopped throwing ${e}"
                )

                if(exceptionsThrown.isEmpty())
                    onPipelineExceptionClear.run()
            }
        }

        onUpdate.run()
    }

    val message: String get() {
        if(currentPipeline == null)
            return "**No pipeline selected**"

        val messageBuilder = StringBuilder()
        val pipelineName = currentPipeline!!.clazz.simpleName

        if(exceptionsThrown.isNotEmpty()) {
            messageBuilder
                .append("**Pipeline $pipelineName is throwing ${exceptionsThrown.size} exception(s)**")
                .appendLine("\n")
        } else {
            messageBuilder
                .append("**Pipeline $pipelineName running OK at ${pipelineManager.pipelineFpsCounter.fps} FPS**")
                .appendLine("\n")
        }

        for((_, data) in exceptionsThrown) {
            val expiresIn = millisExceptionExpire - (System.currentTimeMillis() - data.millisThrown)
            val expiresInSecs = String.format("%.2f", expiresIn.toDouble() / 1000.0)

            val shortStacktrace = StrUtil.cutStringBy(
                data.stacktrace, "at com.github.serivesmejia.eocvsim", 1
            ).trim()

            messageBuilder
                .appendLine("> $shortStacktrace")
                .appendLine()
                .appendLine("! It has been thrown ${data.count} times, and will expire in $expiresInSecs seconds !")
                .appendLine()
        }

        return messageBuilder.toString().trim()
    }

    data class PipelineException(var count: Int,
                                 val stacktrace: String,
                                 var millisThrown: Long)

}