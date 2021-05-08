#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"																																		
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "softdevice_handler.h"
#include "app_timer.h"																															//
#include "ble_nus.h"																																//
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"																																
#include "boards.h"
#include "twi_master.h"																															//
#include "mpu6050.h"																																//MPU6050驱动 
#include "inv_mpu.h"																																//MPU6050硬件DMP 
#include "inv_mpu_dmp_motion_driver.h"																							//MPU6050硬件DMP驱动 


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART"                               //预设蓝牙名称 

#define APP_ADV_INTERVAL                32                                          //广播间隔，以0.625ms为单位 
#define APP_ADV_TIMEOUT_IN_SECONDS      0 																					//广播超时，0为无超时 

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         5                                           //盘牙定时器队列 

#define SLAVE_LATENCY                   0                                           //

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SlaveNO													0x50

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

APP_TIMER_DEF(update_timer);																												//RTC时钟空间宏申请 

int16_t AccValue[3],GyroValue[3];
uint8_t id;

float pitch,roll,yaw; 																															//俯仰、横滚、偏航 
short aacx,aacy,aacz;		 																														//加速度 
short gyrox,gyroy,gyroz;																														//角速度 

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)						//回调函数，错误处理 
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void sleep_mode_enter(void)																									//低功耗深度睡眠模式 
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)																		//广播事件处理函数 
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);								//蓝牙广播标识位 
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

static void on_ble_evt(ble_evt_t * p_ble_evt)																				//低功耗蓝牙事件处理函数 
{
    uint32_t err_code;

	switch (p_ble_evt->header.evt_id)
	{

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)																	//蓝牙事件分发函数 
{
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}

static void ble_stack_init(void)																										//协议栈初始化函数 
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


void bsp_event_handler(bsp_event_t event)																						//蓝牙BSP事件处理函数 
{
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


ble_advdata_manuf_data_t manuf_data;																								//厂商自定义，在函数外保证全局通用

uint8_t mpu_6050_data[14] = {SlaveNO,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


static void advertising_init(void)																									//定义广播初始化函数 
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;
	
	//自定义参数
		int8_t tx_power = 0; 
		manuf_data.company_identifier  = 0x0059;//0x004C;																//设置厂商标识，0x0059为Nodic 
		manuf_data.data.p_data = mpu_6050_data;																					//设置自定义广播数据数组头读取地址 
		manuf_data.data.size = sizeof(mpu_6050_data);																		//设置自定义广播数据长度 

    memset(&advdata, 0, sizeof(advdata));																						//设置广播参数接入结构体 
    advdata.name_type          = BLE_ADVDATA_NO_NAME;																//配置蓝牙广播不包含蓝牙名称 
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;				//设置蓝牙广播标识，为常规不限时BLE广播 
		advdata.p_tx_power_level = &tx_power;																						//配置广播发射功率等级，常规参数有0dBm、+4dBm，+20dBm 
		advdata.p_manuf_specific_data = &manuf_data;


    memset(&scanrsp, 0, sizeof(scanrsp));

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;																						//快速广播使能 
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;																//配置广播间隔 
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;											//配置广播超时 

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);//底层广播初始化 
    APP_ERROR_CHECK(err_code);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void advdata_Update(void *p_contex)																									//广播数据更新函数 
{
		uint8_t i;
	
		AccValue[0] = aacx;																															//加速度 
		AccValue[1] = aacy;
		AccValue[2] = aacz;
		
		GyroValue[0] = (int)(roll*100);																									//欧拉角覆盖原角速度 
		GyroValue[1] = (int)(pitch*100);
		GyroValue[2] = (int)(yaw*10);
		
		mpu_6050_data[1] += 1;																													//广播包序号标识，防主机复读 
		for(i=2;i<7;i+=2)																																//加速度广播数据区域填充 
		{
			mpu_6050_data[i]=AccValue[i/2-1]>>8;																					//按高低字节填充数组 
			mpu_6050_data[i+1]=AccValue[i/2-1]&0x00ff;
		}
		for(i=8;i<13;i+=2)																															//欧拉角广播数据区域填充 
		{
			mpu_6050_data[i]=GyroValue[i/2-4]>>8;																					//按高低字节填充数组 
			mpu_6050_data[i+1]=GyroValue[i/2-4]&0x00ff;
		}

		sd_ble_gap_adv_stop();																													//停止广播 
		
		advertising_init();																															//重载广播数据，初始化
		
		ble_advertising_start(BLE_ADV_MODE_FAST);																				//开启广播 
}

void Read_Mpu6050()																																	//读MPU6050传感器，DMP计算欧拉角 
{
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)																				//根据速度数据计算欧拉角 
	{
		MPU6050_ReadAcc(&aacx,&aacy,&aacz);	    																				//读取加速度 
		MPU6050_ReadGyro(&gyrox,&gyroy,&gyroz);																					//读取角加速度 
		nrf_delay_ms(5);																																//读取间隔 
	}
}

int main(void)
{
    // Initialize.
		APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);						//RTC时钟初始化 
		app_timer_create(&update_timer,APP_TIMER_MODE_REPEATED,advdata_Update);					//创建广播数据更新函数 

		ble_stack_init();																																//协议栈初始化
		twi_master_init();																															//IIC通信端口初始化 

		do
			nrf_delay_ms(1000);
		while(mpu6050_init(0x68) == false);																							//MPU6050初始化 
		mpu6050_register_read(0x75U, &id, 1);																						
		
		do
			nrf_delay_ms(1000);
		while(mpu_dmp_init());																													//DMP初始化 
		Read_Mpu6050();																																	//读取数据 
		
		if(mpu_6050_data[0] == 0x54)nrf_delay_ms(5000);																	//
		
		app_timer_start(update_timer, APP_TIMER_TICKS(80,APP_TIMER_PRESCALER), NULL);		//开启定时器 

    for (;;)
    {
			Read_Mpu6050();																																//读取MPU6050数据，计算欧拉角 
			power_manage();																																//电源管理 
    }
}
