/******************************************************************************
 *
 *  Copyright (C) 2003-2012 Broadcom Corporation
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

/******************************************************************************
 *
 *  This is the interface file for advanced audio call-out functions.
 *
 ******************************************************************************/
#ifndef BTA_AV_CO_H
#define BTA_AV_CO_H

#include "bta_av_api.h"
#include "l2c_api.h"

/*****************************************************************************
 *  Constants and data types
 ****************************************************************************/

/*******************************************************************************
 *
 * Function         bta_av_co_audio_init
 *
 * Description      This callout function is executed by AV when it is
 *                  started by calling BTA_AvEnable().  This function can be
 *                  used by the phone to initialize audio paths or for other
 *                  initialization purposes.
 *
 *
 * Returns          Stream codec and content protection capabilities info.
 *
 ******************************************************************************/
bool bta_av_co_audio_init(btav_a2dp_codec_index_t codec_index,
                          tAVDT_CFG* p_cfg);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_disc_res
 *
 * Description      This callout function is executed by AV to report the
 *                  number of stream end points (SEP) were found during the
 *                  AVDT stream discovery process.
 *
 *
 * Returns          void.
 *
 ******************************************************************************/
void bta_av_co_audio_disc_res(tBTA_AV_HNDL hndl, uint8_t num_seps,
                              uint8_t num_snk, uint8_t num_src, BD_ADDR addr,
                              uint16_t uuid_local);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_getconfig
 *
 * Description      This callout function is executed by AV to retrieve the
 *                  desired codec and content protection configuration for the
 *                  audio stream.
 *
 *
 * Returns          Stream codec and content protection configuration info.
 *
 ******************************************************************************/
tA2DP_STATUS bta_av_co_audio_getconfig(tBTA_AV_HNDL hndl, uint8_t* p_codec_info,
                                       uint8_t* p_sep_info_idx, uint8_t seid,
                                       uint8_t* p_num_protect,
                                       uint8_t* p_protect_info);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_setconfig
 *
 * Description      This callout function is executed by AV to set the
 *                  codec and content protection configuration of the audio
 *                  stream.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_setconfig(tBTA_AV_HNDL hndl, const uint8_t* p_codec_info,
                               uint8_t seid, BD_ADDR addr, uint8_t num_protect,
                               const uint8_t* p_protect_info,
                               uint8_t t_local_sep, uint8_t avdt_handle);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_open
 *
 * Description      This function is called by AV when the audio stream
 *                  connection is opened.
 *                  BTA-AV maintains the MTU of A2DP streams.
 *                  If this is the 2nd audio stream, mtu is the smaller of the 2
 *                  streams.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_open(tBTA_AV_HNDL hndl, uint16_t mtu);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_close
 *
 * Description      This function is called by AV when the audio stream
 *                  connection is closed.
 *                  BTA-AV maintains the MTU of A2DP streams.
 *                  When one stream is closed and no other audio stream is open,
 *                  mtu is reported as 0.
 *                  Otherwise, the MTU remains open is reported.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_close(tBTA_AV_HNDL hndl);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_start
 *
 * Description      This function is called by AV when the audio streaming data
 *                  transfer is started.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_start(tBTA_AV_HNDL hndl, uint8_t* p_codec_info,
                           bool* p_no_rtp_hdr);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_stop
 *
 * Description      This function is called by AV when the audio streaming data
 *                  transfer is stopped.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_stop(tBTA_AV_HNDL hndl);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_src_data_path
 *
 * Description      This function is called to get the next data buffer from
 *                  the audio codec
 *
 * Returns          NULL if data is not ready.
 *                  Otherwise, a buffer (BT_HDR*) containing the audio data.
 *
 ******************************************************************************/
void* bta_av_co_audio_src_data_path(const uint8_t* p_codec_info,
                                    uint32_t* p_timestamp);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_drop
 *
 * Description      An Audio packet is dropped. .
 *                  It's very likely that the connected headset with this handle
 *                  is moved far away. The implementation may want to reduce
 *                  the encoder bit rate setting to reduce the packet size.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_drop(tBTA_AV_HNDL hndl);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_delay
 *
 * Description      This function is called by AV when the audio stream
 *                  connection needs to send the initial delay report to the
 *                  connected SRC.
 *
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_delay(tBTA_AV_HNDL hndl, uint16_t delay);

/*******************************************************************************
 *
 * Function         bta_av_co_audio_update_mtu
 *
 * Description      This function is called by AV when the audio stream
 *                  connection MTU needs to be updated.
 *                  BTA-AV maintains the MTU of A2DP streams.
 *                  If this is the 2nd audio stream, mtu is the smaller of the 2
 *                  streams.
 *
 * Returns          void
 *
 ******************************************************************************/
void bta_av_co_audio_update_mtu(tBTA_AV_HNDL hndl, uint16_t mtu);
/*******************************************************************************
 **
 ** Function         bta_av_co_cp_get_flag
 **
 ** Description      Get content protection flag
 **                  AVDT_CP_SCMS_COPY_NEVER
 **                  AVDT_CP_SCMS_COPY_ONCE
 **                  AVDT_CP_SCMS_COPY_FREE
 **
 ** Returns          The current flag value
 **
 ******************************************************************************/
uint8_t bta_av_co_cp_get_flag(void);

/*******************************************************************************
 **
 ** Function         bta_av_co_cp_is_active
 **
 ** Description     Get the current configuration of content protection
 **
 ** Returns          TRUE if the current streaming has CP, FALSE otherwise
 **
 ******************************************************************************/
bool bta_av_co_cp_is_active(void);

#endif /* BTA_AV_CO_H */
