/*
 * Copyright Telechips Inc.
 *
 * TCC Version 1.0
 *
 * This source code contains confidential information of Telechips.
 *
 * Any unauthorized use without a written permission of Telechips including not
 * limited to re-distribution in source or binary form is strictly prohibited.
 *
 * This source code is provided "AS IS" and nothing contained in this source code
 * shall constitute any express or implied warranty of any kind, including without
 * limitation, any warranty of merchantability, fitness for a particular purpose
 * or non-infringement of any patent, copyright or other third party intellectual
 * property right.
 * No warranty is made, express or implied, regarding the information's accuracy,
 * completeness, or performance.
 *
 * In no event shall Telechips be liable for any claim, damages or other
 * liability arising from, out of or in connection with this source code or
 * the use in the source code.
 *
 * This source code is provided subject to the terms of a Mutual Non-Disclosure
 * Agreement between Telechips and Company.
 */

#ifndef __NNAPPMAIN_H__
#define __NNAPPMAIN_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <math.h>
#include <execinfo.h> // to use backtrace
#include <signal.h> // to use sigaction
#include <pthread.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <stdbool.h>
#include <tcc_scaler_ioctrl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdbool.h>

#include "NnType.h"
#include "NnDebug.h"
#include "NnRtpm.h"
#include "NnPerf.h"
#include "NnMemory.h"
#include "NnNeuralNetwork.h"

#include "camera_api.h"
#include "display_api.h"
#include "scaler_api.h"
#include "message_api.h"

#include "NnSignalHandler.h"

#include "opencv_api.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
typedef struct param_info
{
	char *networkPath[NETWORK_INDEX_MAX];
	uint8_t networkCnt;
	char* inputPath;
	char* outputPath;
	uint16_t inputWidth;
	uint16_t inputHeight;
	image_fmt_t inputFormat;
	uint16_t outputWidth;
	uint16_t outputHeight;
	image_fmt_t outputFormat;
	uint8_t npuIdx[NPU_CLUSTER_INDEX_MAX];
	input_data_type_t inputMode;
	output_data_type_t outputMode;
	npu_debug_mode_t npuDebugMode;
	debug_mode_t debugMode;
	npu_run_mode_t npuRunMode;
	encoding_mode_t encodingMode;
	int32_t encodingTime;
	uint32_t outputSx;
	uint32_t outputSy;
	char* pRtpmIpAdress;
} param_info_t;

typedef struct app_context
{
	//npu & neural network
	inference_context_t inferenceContext;

	//memory
	memory_context_t memory_context;
	uint64_t phy_base_input;
	uint8_t *map_base_input;
	uint32_t phy_base_output_idx;

	//input
	input_data_type_t inputDataType;
	uint32_t inputWidth;
	uint32_t inputHeight;
	image_fmt_t inputImageFormat;
	char* inputPath;

	//output
	output_data_type_t outputDataType;
	uint32_t outputWidth;
	uint32_t outputHeight;
	image_fmt_t outputImageFormat;
	scaler_index_t outputScalerIdx;
	char* outputPath;

	encoding_mode_t encodingMode;

	//input > camera
	uint8_t camCaptureRetryCnt;
	//input > file
	//input > rtpm
	uint8_t inputRtpmBufferCount;
	uint32_t inputRtpmBufferIndex;
	char* pRtpmIpAdress;
	uint16_t streamPort;
	uint16_t messagePort;

	//output > lcd
	uint32_t outputLcdSx;
	uint32_t outputLcdSy;

	//output > file

	//output > rtpm
	uint8_t outputRtpmBufferCount;

	//system info
	perf_context_t perfContext;

	// debug
	debug_mode_t debugMode;

}app_context_t;

#endif //__NNAPPMAIN_H__
