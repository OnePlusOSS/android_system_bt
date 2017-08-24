/******************************************************************************
 * Copyright (C) 2017, The Linux Foundation. All rights reserved.
 *
 *  Not a Contribution
 *****************************************************************************/
/******************************************************************************
 *
 *  Copyright (C) 2017 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "btif_a2dp_audio_interface"

//#include "audio_a2dp_hw.h"
#include "btif_a2dp_audio_interface.h"
#include "bt_common.h"
#include "btif_a2dp.h"
#include "btif_a2dp_control.h"
#include "btif_a2dp_sink.h"
#include "btif_a2dp_source.h"
#include "btif_av.h"
#include "btif_av_co.h"
#include "btif_hf.h"
#include "osi/include/osi.h"
#include <base/logging.h>
#include <utils/RefBase.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/IBluetoothAudio.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/IBluetoothAudioCallbacks.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/types.h>
#include <hwbinder/ProcessState.h>

using com::qualcomm::qti::bluetooth_audio::V1_0::IBluetoothAudio;
using com::qualcomm::qti::bluetooth_audio::V1_0::IBluetoothAudioCallbacks;
using com::qualcomm::qti::bluetooth_audio::V1_0::Status;
using com::qualcomm::qti::bluetooth_audio::V1_0::CodecCfg;
using android::hardware::ProcessState;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::hardware::hidl_vec;
using ::android::sp;
android::sp<IBluetoothAudio> btAudio;

#define CASE_RETURN_STR(const) \
  case const:                  \
    return #const;

uint8_t codec_info[30];
uint8_t len,a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
Status mapToStatus(uint8_t resp);
uint8_t btif_a2dp_audio_process_request(uint8_t cmd);

/*BTIF AV helper */
extern bool btif_av_is_device_disconnecting();
extern bool reconfig_a2dp;
#if 0
typedef enum {
  A2DP_CTRL_GET_CODEC_CONFIG = 15,
  A2DP_CTRL_GET_MULTICAST_STATUS,
  A2DP_CTRL_GET_CONNECTION_STATUS,
  A2DP_CTRL_GET_NUM_CONNECTED_DEVICE,
}tA2DP_CTRL_EXT_CMD;
typedef enum {
  A2DP_CTRL_ACK_PENDING = 5,
}tA2DP_CTRL_ACK_EXT;
#endif
#ifdef BTA_AV_SPLIT_A2DP_DEF_FREQ_48KHZ
#define A2DP_SBC_DEFAULT_BITRATE 345

#ifndef A2DP_SBC_NON_EDR_MAX_RATE
#define A2DP_SBC_NON_EDR_MAX_RATE 237
#endif
#else
#define A2DP_SBC_DEFAULT_BITRATE 328

#ifndef A2DP_SBC_NON_EDR_MAX_RATE
#define A2DP_SBC_NON_EDR_MAX_RATE 229
#endif
#endif

class BluetoothAudioCallbacks : public IBluetoothAudioCallbacks {
 public:
    Return<Status> a2dp_start_stream_req() {
        uint8_t resp;
        LOG_INFO(LOG_TAG,"a2dp_start_stream_req");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_START);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        return mapToStatus(resp);
        //return ::android::hardware::bluetooth_audio::V1_0::Status {};
    }
    Return<Status> a2dp_suspend_stream_req() {
        uint8_t resp;
        LOG_INFO(LOG_TAG,"a2dp_suspend_stream_req");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_SUSPEND);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        return mapToStatus(resp);
//        return ::android::hardware::bluetooth_audio::V1_0::Status {};
    }
    Return<Status> a2dp_stop_stream_req() {
        uint8_t resp;
        LOG_INFO(LOG_TAG,"a2dp_stop_stream_req");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_STOP);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        return mapToStatus(resp);
//        return ::android::hardware::bluetooth_audio::V1_0::Status {};
    }
    Return<Status> a2dp_check_ready() {
        uint8_t resp;
        LOG_INFO(LOG_TAG,"a2dp_check_ready");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_CHECK_READY);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        return mapToStatus(resp);
//        return ::android::hardware::bluetooth_audio::V1_0::Status {};
    }
    using get_codec_config_cb = std::function<void(Status status, const hidl_vec<uint8_t>& cfg)>;
    Return<void> a2dp_get_codec_config(get_codec_config_cb _hidl_cb) {
        uint8_t resp;
        CodecCfg data;
        LOG_INFO(LOG_TAG,"a2dp_get_codec_config");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_GET_CODEC_CONFIG);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        data.setToExternal(codec_info,len);
        _hidl_cb(mapToStatus(resp), data);
        return Void();
    }
    Return<uint8_t> a2dp_get_multicast_status() {
        uint8_t ret = 0;
        //TODO implement
        LOG_INFO(LOG_TAG,"a2dp_get_multicast_status");
        return ret;
    }
    Return<uint8_t> a2dp_get_num_connected_devices() {
        uint8_t ret = 1;
        //TODO implement
        LOG_INFO(LOG_TAG,"a2dp_get_num_connected_devices");
        return ret;
    }
    Return<Status> a2dp_get_connection_status() {
        uint8_t resp;
        LOG_INFO(LOG_TAG,"a2dp_get_connection_status");
        resp = btif_a2dp_audio_process_request(A2DP_CTRL_GET_CONNECTION_STATUS);
        LOG_INFO(LOG_TAG,"resp  = %d",resp);
        return mapToStatus(resp);
        //return ::android::hardware::bluetooth_audio::V1_0::Status {};
    }
    Return<uint16_t> a2dp_get_sink_latency() {
        uint16_t sink_latency;
        LOG_INFO(LOG_TAG,"a2dp_get_sink_latency");

        sink_latency = btif_av_get_sink_latency();
        return sink_latency;
    }
};

Status mapToStatus(uint8_t resp)
{
    switch(resp) {
      case A2DP_CTRL_ACK_SUCCESS:
        return Status::SUCCESS;
        break;
      case A2DP_CTRL_ACK_FAILURE:
        return Status::FAILURE;
        break;
      case A2DP_CTRL_ACK_INCALL_FAILURE:
        return Status::INCALL_FAILURE;
        break;
      case A2DP_CTRL_ACK_PENDING:
        return Status::PENDING;
        break;
      case A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS:
        return Status::DISCONNECTING;
      default:
        LOG_INFO(LOG_TAG,"Invalid Status");
        return Status::UNSUPPORTED;
        break;
    }
  return Status::SUCCESS;
}
void btif_a2dp_audio_interface_init() {
  btAudio = IBluetoothAudio::getService();
  CHECK(btAudio != nullptr);

  LOG_INFO(LOG_TAG, "%s: IBluetoothAudio::getService() returned %p (%s)",
           __func__, btAudio.get(), (btAudio->isRemote() ? "remote" : "local"));
  {
    LOG_INFO(LOG_TAG,"%s:Calling Init",__func__);
    android::sp<IBluetoothAudioCallbacks> callbacks = new BluetoothAudioCallbacks();
    btAudio->initialize_callbacks(callbacks);
  }
  LOG_INFO(LOG_TAG,"%s:Init returned",__func__);
}
void btif_a2dp_audio_interface_deinit() {
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_interface_deinit");
  if (btAudio != nullptr) {
    auto ret = btAudio->deinitialize_callbacks();
    if (!ret.isOk()) {
      LOG_ERROR(LOG_TAG,"hal server is dead");
    }
  }
  btAudio = nullptr;
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_interface_deinit:Exit");
}
void btif_a2dp_audio_on_started(tBTA_AV_STATUS status)
{
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : status = %d",status);
  if (btAudio != nullptr){
    if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
      LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
      btAudio->a2dp_on_started(mapToStatus(status));
    }
  }
}

void btif_a2dp_audio_on_suspended(tBTA_AV_STATUS status)
{
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : status = %d",status);
  if (btAudio != nullptr) {
    if (a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND) {
      LOG_INFO(LOG_TAG,"calling method a2dp_on_suspended");
      btAudio->a2dp_on_suspended(mapToStatus(status));
    }
  }
}

void btif_a2dp_audio_on_stopped(tBTA_AV_STATUS status)
{
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : status = %d",status);
  if (btAudio != nullptr){
    if (a2dp_cmd_pending == A2DP_CTRL_CMD_STOP) {
      LOG_INFO(LOG_TAG,"calling method a2dp_on_stopped");
      btAudio->a2dp_on_stopped(mapToStatus(status));
    } else if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
      LOG_INFO(LOG_TAG,"Remote disconnected when start under progress");
      btAudio->a2dp_on_started(mapToStatus(A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
    }
  }
}
uint8_t btif_a2dp_audio_process_request(uint8_t cmd)
{
  APPL_TRACE_DEBUG(LOG_TAG,"btif_a2dp_audio_process_request %s", audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
  a2dp_cmd_pending = cmd;
  uint8_t status;
  switch (cmd) {
    case A2DP_CTRL_CMD_CHECK_READY:
      if (btif_a2dp_source_media_task_is_shutting_down()) {
        APPL_TRACE_WARNING("%s: A2DP command %s while media task shutting down",
                           __func__,audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
        //btif_a2dp_command_ack(A2DP_CTRL_ACK_FAILURE);
        status = A2DP_CTRL_ACK_FAILURE;
        break;
      }
      if (!btif_hf_is_call_vr_idle()) {
        status  = A2DP_CTRL_ACK_INCALL_FAILURE;
        break;
      }
      if (btif_av_is_under_handoff() || reconfig_a2dp) {
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
      /* check whether AV is ready to setup A2DP datapath */
      if (btif_av_stream_ready() || btif_av_stream_started_ready()) {
        status = A2DP_CTRL_ACK_SUCCESS;
      } else {
        APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                           __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
        status = A2DP_CTRL_ACK_FAILURE;
      }
      break;

    case A2DP_CTRL_CMD_START:
      /*
       * Don't send START request to stack while we are in a call.
       * Some headsets such as "Sony MW600", don't allow AVDTP START
       * while in a call, and respond with BAD_STATE.
       */
      if (!btif_hf_is_call_vr_idle()) {
        status = A2DP_CTRL_ACK_INCALL_FAILURE;
        break;
      }

      if (btif_a2dp_source_is_remote_start()) {
        APPL_TRACE_WARNING("%s: remote a2dp started, cancel remote start timer",
                           __func__);
        btif_a2dp_source_cancel_remote_start();
        btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }

      /* In dual a2dp mode check for stream started first*/
      if (btif_av_stream_started_ready()) {
        /*
         * Already started, setup audio data channel listener and ACK
         * back immediately.
         */
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
      if (btif_av_stream_ready()) {
        /*
         * Post start event and wait for audio path to open.
         * If we are the source, the ACK will be sent after the start
         * procedure is completed, othewise send it now.
         */
        btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
        int idx = btif_av_get_latest_device_idx_to_start();
        if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SRC) {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        /*Return pending and ack when start stream cfm received from remote*/
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }

      APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                         __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
      return A2DP_CTRL_ACK_FAILURE;
      break;

    case A2DP_CTRL_CMD_STOP:
      {
        int idx = btif_av_get_latest_playing_device_idx();
        if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SNK &&
            !btif_a2dp_source_is_streaming()) {
          /* We are already stopped, just ack back */
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }

        btif_dispatch_sm_event(BTIF_AV_STOP_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_SUCCESS;

        break;
      }
    case A2DP_CTRL_CMD_SUSPEND:
      /* Local suspend */
      if (reconfig_a2dp ||
          btif_a2dp_source_is_remote_start()) {
        LOG_INFO(LOG_TAG,"Suspend called due to reconfig or remote started");
        /*if (btif_av_is_under_handoff() && !btif_av_is_device_disconnecting()) {
          LOG_INFO(LOG_TAG,"Under hand off,hopefully stack send success ack");
          status = A2DP_CTRL_ACK_PENDING;
        } else {
          status = A2DP_CTRL_ACK_SUCCESS;
        }*/
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
      if (btif_av_stream_started_ready()) {
        btif_dispatch_sm_event(BTIF_AV_SUSPEND_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }
      /* If we are not in started state, just ack back ok and let
       * audioflinger close the channel. This can happen if we are
       * remotely suspended, clear REMOTE SUSPEND flag.
       */
      btif_av_clear_remote_suspend_flag();
      status = A2DP_CTRL_ACK_SUCCESS;
      break;

    case A2DP_CTRL_CMD_OFFLOAD_START:
      btif_dispatch_sm_event(BTIF_AV_OFFLOAD_START_REQ_EVT, NULL, 0);
      status = A2DP_CTRL_ACK_PENDING;
      break;
    case A2DP_CTRL_GET_CODEC_CONFIG:
      {
        uint8_t p_codec_info[AVDT_CODEC_SIZE];
        uint8_t codec_type;
        tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
        uint32_t bitrate = 0;
        len = 0;
        LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG");
        A2dpCodecConfig *CodecConfig = bta_av_get_a2dp_current_codec();
        bta_av_co_get_peer_params(&peer_param);
        if (btif_av_stream_started_ready() == FALSE)
        {
            LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG: stream not started");
            status = A2DP_CTRL_ACK_FAILURE;
            break;
        }
        memset(p_codec_info, 0, AVDT_CODEC_SIZE);
        memset(codec_info, 0, 30);
        if (!CodecConfig->copyOutOtaCodecConfig(p_codec_info))
        {
          LOG_INFO(LOG_TAG,"No valid codec config");
          status = A2DP_CTRL_ACK_FAILURE;
          break;
        }
        memcpy(&codec_info[1], &p_codec_info, p_codec_info[0] + 1);

        codec_type = A2DP_GetCodecType((const uint8_t*)p_codec_info);
        LOG_INFO(LOG_TAG,"codec_type = %x",codec_type);
        if (A2DP_MEDIA_CT_SBC == codec_type)
        {
          uint16_t rate = A2DP_SBC_DEFAULT_BITRATE;
          if (!peer_param.is_peer_edr)
            rate = A2DP_SBC_NON_EDR_MAX_RATE;
          bitrate =  rate * 1000;
        }
        else if (A2DP_MEDIA_CT_NON_A2DP == codec_type)
        {
          int bits_per_sample = A2DP_GetTrackBitsPerSample(p_codec_info);
          int samplerate = A2DP_GetTrackSampleRate(p_codec_info);

          /* BR = (Sampl_Rate * PCM_DEPTH * CHNL)/Compression_Ratio */
          bitrate = (samplerate * bits_per_sample * 2)/4;
        }
        else if (A2DP_MEDIA_CT_AAC == codec_type)
        {
          bitrate = 0;//Bitrate is present in codec info
        }
        codec_info[0] = 0; //playing device handle
        len = p_codec_info[0] + 2;
        codec_info[len++] = (uint8_t)(peer_param.peer_mtu & 0x00FF);
        codec_info[len++] = (uint8_t)(((peer_param.peer_mtu & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(bitrate & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF0000) >> 16) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF000000) >> 24) & 0x00FF);
        LOG_INFO(LOG_TAG,"len  = %d", len);
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
    case A2DP_CTRL_GET_CONNECTION_STATUS:
      if (btif_av_is_connected())
      {
          BTIF_TRACE_DEBUG("got valid connection");
          status = A2DP_CTRL_ACK_SUCCESS;
      }
      else
          status = A2DP_CTRL_ACK_FAILURE;
      break;

    case A2DP_CTRL_GET_MULTICAST_STATUS:
/*
        uint8_t playing_devices = (uint8_t)btif_av_get_num_playing_devices();
        BOOLEAN multicast_state = btif_av_get_multicast_state();
        a2dp_cmd_acknowledge(A2DP_CTRL_ACK_SUCCESS);
        multicast_query = FALSE;
        if ((btif_max_av_clients > 1 && playing_devices == btif_max_av_clients) &&
            multicast_state)
        {
            multicast_query = TRUE;
        }
        BTIF_TRACE_ERROR("multicast status = %d",multicast_query);
        UIPC_Send(UIPC_CH_ID_AV_CTRL, 0, &multicast_query, 1);
        UIPC_Send(UIPC_CH_ID_AV_CTRL, 0, &playing_devices, 1);
*/
      status = 0; //FALSE
      break;

    case A2DP_CTRL_GET_NUM_CONNECTED_DEVICE:
      status = 1;
      break;

    default:
      APPL_TRACE_ERROR("UNSUPPORTED CMD (%d)", cmd);
      status = A2DP_CTRL_ACK_FAILURE;
      break;
  }
  APPL_TRACE_DEBUG("a2dp-ctrl-cmd : %s DONE",
                   audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
  return status;
}
/*
const char* dump_ctrl_event(tA2DP_CTRL_CMD_EXT cmd)
{
  switch(cmd) {
    CASE_RETURN_STR(A2DP_CTRL_CMD_CHECK_READY)
    CASE_RETURN_STR(A2DP_CTRL_CMD_START)
    CASE_RETURN_STR(A2DP_CTRL_CMD_STOP)
    CASE_RETURN_STR(A2DP_CTRL_CMD_SUSPEND)
    CASE_RETURN_STR( A2DP_CTRL_GET_CODEC_CONFIG)
    CASE_RETURN_STR(A2DP_CTRL_GET_MULTICAST_STATUS)
    CASE_RETURN_STR(A2DP_CTRL_GET_CONNECTION_STATUS)
    CASE_RETURN_STR(A2DP_CTRL_GET_NUM_CONNECTED_DEVICE)
  }
}*/
