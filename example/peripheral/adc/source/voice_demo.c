/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       voice_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "adc.h"
#include "adc_demo.h"
#include "log.h"
#include "voice.h"
#include "Voice_Queue.h"
#include "mcu.h"

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/


tCircularBuffer* VoiceRaw_FiFO;
static uint8_t voiceLeftBuf[ VOICE_FIFO_BUFF_SIZE ];


// !pushing to master's inqueue buff
uint8 inqueue_buf[ VOICE_REPORT_FRAME_SIZE ]  = { 0x00 };

// !pushing to master's outqueue buff
uint8 outqueue_buf[ VOICE_REPORT_FRAME_SIZE ] = { 0x00 };



/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

uint8 voiceDemo_TaskID;   // Task ID for internal task/event processing
/*
    channel:
    is_differential_mode:
    is_high_resolution:
    [bit7~bit2]=[p20,p15~p11],ignore[bit1,bit0]
    when measure adc(not battery),we'd better use high_resolution.
    when measure battery,we'd better use no high_resolution and keep the gpio alone.

    differential_mode is rarely used,
    if use please config channel as one of [ADC_CH3DIFF,ADC_CH2DIFF,ADC_CH1DIFF],
    and is_high_resolution as one of [0x80,0x20,0x08],
    then the pair of [P20~P15,P14~P13,P12~P11] will work.
    other adc channel cannot work.
*/


/*********************************************************************
    LOCAL FUNCTIONS
*/
static void voice_ProcessOSALMsg( osal_event_hdr_t* pMsg );

// !request temp cache buff
uint8 voice_tmp_buf[ VOICE_REQUEST_CACHE_BUFF_SIZE ]= { 0x00 };


/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

uint8 voice_requeset_data(uint8* buf,uint16 len)
{
    if(len==0)
        return 0;

    if(IsBufferSizeFilled(VoiceRaw_FiFO,len))
    {
        if(ReadBuffer(VoiceRaw_FiFO,buf,len)==len)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}

static void pin_voice_stop_event(GPIO_Pin_e pin,IO_Wakeup_Pol_e type)
{
    stop_voice_task();
}



void voice_Init( uint8 task_id )
{
    voiceDemo_TaskID = task_id;
    VoiceRaw_FiFO=(tCircularBuffer*)osal_mem_alloc(sizeof(tCircularBuffer));
    hal_gpio_pull_set(P1,GPIO_PULL_UP_S);
    hal_gpio_pull_set(P0,GPIO_PULL_DOWN);
    hal_gpioin_register(P0,pin_voice_stop_event,NULL);
}
static void voice_evt_handler_adpcm(voice_Evt_t* pev)
{
    uint8_t leftbuf[2];
    uint8_t rightbuf[2];
    uint8_t left_right_chanle;
    uint32_t voiceSampleDual;
    int voiceSampleRight;
    int voiceSampleLeft;
    uint32_t i=0;
    left_right_chanle = 1;

    if(pev->type == HAL_VOICE_EVT_DATA)
    {
        for(i=0; i < pev->size; i++)
        {
            voiceSampleDual = pev->data[i];
            voiceSampleRight = (int16)(voiceSampleDual & 65535);
            voiceSampleLeft = (int16)((voiceSampleDual >> 16) & 65535);

            if(left_right_chanle==1)
            {
                leftbuf[0]= voiceSampleLeft;
                leftbuf[1]= voiceSampleLeft>>8;
                FillBuffer(VoiceRaw_FiFO, leftbuf, 2);
            }
            else
            {
                rightbuf[0]= voiceSampleRight;
                rightbuf[1]= voiceSampleRight>>8;
                FillBuffer(VoiceRaw_FiFO, rightbuf, 2);
            }
        }

        #if 1

        // 向fifo中请求(VOICE_REPORT_FRAME_SIZE- cutom_frame_head_len )*4个数据
        if(voice_requeset_data(voice_tmp_buf, VOICE_REQUEST_CACHE_BUFF_SIZE))
        {
            osal_set_event(voiceDemo_TaskID, VOICE_ENCODE_EVT);
        }

        #else
        #endif
    }
}


static void voiceCaptureInit( void )
{
    voice_Cfg_t cfg;
    //
    cfg.voiceSelAmicDmic = 0;
    // !DMIC,dmicDataPin&dmicClkPin
    cfg.dmicDataPin = VOICE_DATA_GPIO;
    cfg.dmicClkPin = VOICE_CLK_GPIO;
    // !amic gain config, default value 0
    cfg.amicGain = 0x02;
    // !voicegain config
    cfg.voiceGain = 40;
    // !
    cfg.voiceEncodeMode = VOICE_ENCODE_BYP;
    // !voice collect rate,default is 8K
    cfg.voiceRate = VOICE_COLLECT_RATE;
    // !open mute
    cfg.voiceAutoMuteOnOff = 1;
    hal_gpio_pin_init(P20, IE);
    hal_gpio_pull_set(P20, FLOATING);
    // !voice hal config
    volatile int voiceConfigStatus = hal_voice_config(cfg, voice_evt_handler_adpcm);

    if(voiceConfigStatus)
    {
        LOG("[Voice]:configuration no:%d\n",voiceConfigStatus);
        return;
    }
    else
    {
//        LOG("[Voice]:start\n");
    }
}

static void voiceCaptureStart( void )
{
    InitQueue();
    VoiceSend_SubIndex=0;
    // !close adc pwrmgr module
    hal_poilling_adc_stop();
    // !voice pwrmgr register
    hal_voice_init();
    //local_count = 0;
    voiceCaptureInit();
    write_reg(&(AP_AON->PMCTL2_1),0);
    InitCircularBuffer(VoiceRaw_FiFO,voiceLeftBuf, VOICE_FIFO_BUFF_SIZE, VOICE_REQUEST_CACHE_BUFF_SIZE );
    hal_voice_start();
}

void voice_data_encode_inqueue(void)
{
    for( uint16 i = 0; i < VOICE_REQUEST_CACHE_BUFF_SIZE; i++ )
    {
        //l_uart_send_buff(UART0, &voice_tmp_buf[i], 1);
        LOG("%02X ", voice_tmp_buf[i]);
    }
}

static void voiceCaptureStop( void )
{
    hal_voice_stop();
    hal_voice_deinit();
    InitCircularBuffer(VoiceRaw_FiFO,voiceLeftBuf, VOICE_FIFO_BUFF_SIZE, VOICE_REQUEST_CACHE_BUFF_SIZE);
    InitQueue();
    VoiceSend_SubIndex=0;
    // collect battdetected after 5 seconds
    osal_start_timerEx(voiceDemo_TaskID, adcMeasureTask_EVT, 5 * 1000);
}

void stop_voice_task(void)
{
    osal_stop_timerEx(voiceDemo_TaskID, VOICE_OUTQUEUE_EVT);
    voiceCaptureStop();
    // notify app audio pushing over
    osal_start_timerEx(voiceDemo_TaskID, VOICE_RECORD_STOP_EVT, 200);
}
//void voice_data_pushing_handle(void)
//{
//    static uint8 outqueue_flag = 0;
//
//  #if ( VOICE_FRAME_MODE ==  VOICE_FRAME_096_DATA)
//  if( VoiceSend_SubIndex == 0 )
//  {
//        if( !outqueue_flag )
//        {
//            if( OutQueue(outqueue_buf) == 1 )
//            {
//                return;
//            }
//        }
//        else
//        {
//            outqueue_flag = 0;
//        }

//  }

//  for(  ; VoiceSend_SubIndex < 5; )
//  {
//      {
//          if( AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 20,&outqueue_buf[VoiceSend_SubIndex*20]) == PPlus_SUCCESS)
//          {
//              VoiceSend_SubIndex++;
//              if( VoiceSend_SubIndex == 5 )
//              {
//                  VoiceSend_SubIndex = 0;
//                  break;
//              }
//
//          }
//          else
//          {
//                if( VoiceSend_SubIndex == 0 )
//                {
//                    outqueue_flag = 1;
//                }
//              break;
//          }
//      }
//
//  }
//
//  #elif( VOICE_FRAME_MODE ==  VOICE_FRAME_128_DATA )
//
//  if( VoiceSend_SubIndex == 0 )
//  {
//        if( !outqueue_flag )
//        {
//            if( OutQueue(outqueue_buf) == 1 )
//            {
//                return;
//            }
//        }
//        else
//        {
//            outqueue_flag = 0;
//        }

//  }

//  for(  ; VoiceSend_SubIndex < 7; )
//  {
//      if( VoiceSend_SubIndex == 6 )
//      {
//          if( AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 14,&outqueue_buf[VoiceSend_SubIndex*20]) == PPlus_SUCCESS )
//          {
//              VoiceSend_SubIndex = 0;
//              break;
//          }
//          else
//          {
//              break;
//          }
//      }
//      else
//      {
//          if( AudioProfile_SetParameter(AUDIOPROFILE_CHAR2, 20,&outqueue_buf[VoiceSend_SubIndex*20]) == PPlus_SUCCESS)
//          {
//              VoiceSend_SubIndex++;
//          }
//          else
//          {
//                if( VoiceSend_SubIndex == 0 )
//                {
//                    outqueue_flag = 1;
//                }
//              break;
//          }
//      }
//
//  }
//    #elif( VOICE_FRAME_MODE ==  VOICE_FRAME_128_DATA_01 )

//    // ! get compression voice data from queue
//    if( VoiceSend_SubIndex == 0 )
//    {
//        if( OutQueue(outqueue_buf) == 1 )
//        {
//            return;
//        }
//        else
//        {
//            outqueue_flag = 0;
//        }
//    }
//
//  // ! update VoiceSend_SubIndex
//  if( VoiceSend_SubIndex == 4 )
//  {
//      VoiceSend_SubIndex = 0;
//  }
//
//  // ! pushing ble data to televisions
//  for( ;VoiceSend_SubIndex < 4; )
//  {
//      // ! pushing compress voice data to audio char
//      if( AudioProfile_SetParameter( AUDIOPROFILE_CHAR2, 134, outqueue_buf ) == PPlus_SUCCESS )
//      {
//          VoiceSend_SubIndex++;
//          // !again get compressing data from queue
//          if( OutQueue(outqueue_buf) == 1 )
//          {
//              // !queue no data
//              VoiceSend_SubIndex = 0;
//              break;
//          }
//          else
//          {
//                // !queue have data continue for circle
//          }
//      }
//      else
//      {
//          // ! pushing data fail wait next event
//          break;
//      }
//  }
//

//  #else
//
//  #endif

//}



static void voice_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
}
uint16 voice_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    //LOG("adc_ProcessEvent: 0x%x\n",events);

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( voiceDemo_TaskID )) != NULL )
        {
            voice_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & VOICE_RECORD_START_EVT )
    {
        voiceCaptureStart();
        return (events ^ VOICE_RECORD_START_EVT);
    }

    if(  events & VOICE_ENCODE_EVT )
    {
        voice_data_encode_inqueue();
        return ( events ^ VOICE_ENCODE_EVT );
    }

    if(  events & VOICE_VOICE_RECORD_STOP_EVT )
    {
        stop_voice_task();
        return ( events ^ VOICE_VOICE_RECORD_STOP_EVT );
    }

    if( events & VOICE_OUTQUEUE_EVT )
    {
        // voice_data_pushing_handle();
        return (events ^ VOICE_OUTQUEUE_EVT);
    }

    if ( events & VOICE_RECORD_STOP_EVT )
    {
//      uint8 notifyencode_value = 0x00;
//      for( uint8 resend_cnt = 0; resend_cnt < 100; resend_cnt++ )
//      {
//          if( AudioProfile_SetParameter(AUDIOPROFILE_CHAR1, sizeof(uint8), &notifyencode_value) == PPlus_SUCCESS)
//          {
//              break;
//          }
//          else
//          {
//              LOG("[Voice]:resend\n");
//          }
//      }
//
//      start_trans_flag = 0;
//      header_index = 0;
////        LOG("[voice]:end\n");
//      osal_start_timerEx( voice_TaskID, SYSTEM_CLK_SWITCH_EVT, 1000);
//        return (events ^ VOICE_RECORD_STOP_EVT);
    }

    // Discard unknown events
    return 0;
}




