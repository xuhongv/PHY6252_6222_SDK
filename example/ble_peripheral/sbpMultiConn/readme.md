# SBP Mulit-Connection Porting Guide

## sbpMulitConn.uvprojx

 1. 修改宏定义 MAX_NUM_LL_CONN=2，配置支持的多slave的个数
 2. 替换 ble_host.lib , 用..\lib\ble_host_multi5.lib 替换。
 3. 替换 peripheral.c, 用  ..\components\profiles\Roles\peripheralMultiConn.c 代替。 
 4. 替换 peripheral.h  用  ..\components\profiles\Roles\peripheralMultiConn.h 代替。 需要替换相关引用的地方。
---

## scatter_load.sct

	peripheral.o(+RO) 替换成 peripheralMultiConn.o(+RO)
---

## main.c
```c
	#define   BLE_MAX_ALLOW_CONNECTION              2  //配置成需要支持的link 个数
```

``` c
int  main(void)
{
    ...
    init_config();
    extern void ll_patch_multi(void);           // 需要替换成 ll_patch_multi
    ll_patch_multi();
    ....
}
```
---

## peripheralMultiConn.h

	配置GAPROLE支持的最大连接数，建议和MAX_NUM_LL_CONN 一致

```c
#define GAPROLE_PERIPHERAL_MAX_CONN              (2)  // maxium allowed connection link
#if(MAX_NUM_LL_CONN<GAPROLE_PERIPHERAL_MAX_CONN)
#error "GAPROLE_PERIPHERAL_MAX_CONN smaller than MAX_NUM_LL_CONN"
#endif
```

---

## peripheralMultiConn.c

	1.增加 connect handle 获取api，可以获取当前active的link 个数，以及当前 active的 conn_handle

```c
bStatus_t GAPRole_GetParameter( uint16 param, void* pValue )
{
 ...
    case GAPROLE_CONNHANDLE:
    {
        uint16* p=(uint16*)pValue;
        for (uint8 i=0;i<GAPROLE_PERIPHERAL_MAX_CONN;i++)
        {
            if(gapRole_ConnectionHandle[i]!=INVALID_CONNHANDLE)
                *p++ = gapRole_ConnectionHandle[i];
        }
    }
        break;
        
    case GAPROLE_CONNECTION_ACTIVE_NUM:
               
        *((uint8*)pValue) = GAPRole_Connect_Active_Num();
        break;
        
 ...
}
```

	2. 增加MulitConn GetParameter接口， 增加入参 connId，获取对应的连接参数和peer mac addr

```c
bStatus_t GAPRole_GetParameter_MultiConn(uint16 connId, uint16 param, void* pValue )
{
    bStatus_t ret = SUCCESS;

    if(connId>GAPROLE_PERIPHERAL_MAX_CONN)
         return bleInvalidRange;

    if(gapRole_ConnectionHandle[connId]==INVALID_CONNHANDLE)
         return bleNotConnected;
 
    switch ( param )
    {
    case GAPROLE_CONN_BD_ADDR:
        VOID osal_memcpy( pValue, gapRole_ConnectedDevAddr[connId], B_ADDR_LEN ) ;
        break;

    case GAPROLE_CONN_INTERVAL:
        *((uint16*)pValue) = gapRole_ConnInterval[connId];
        break;

    case GAPROLE_CONN_LATENCY:
        *((uint16*)pValue) = gapRole_ConnSlaveLatency[connId];
        break;

    case GAPROLE_CONN_TIMEOUT:
        *((uint16*)pValue) = gapRole_ConnTimeout[connId];
        break;

    default:
        ret = INVALIDPARAMETER;

        break;
    }
    return ( ret );
}
```
	3. 增加 GAPRole Set Param api， GAPROLE_PARAM_UPDATE_CONN_HANDLE， 用于发起对应connhandler的连接参数更新
```c
bStatus_t GAPRole_SetParameter( uint16 param, uint8 len, void* pValue )
{
	...
   case GAPROLE_PARAM_UPDATE_CONN_HANDLE:
        if ( (len == sizeof ( uint16 )) && (*((uint16*)pValue) != INVALID_CONNHANDLE) )
        {
            gapRole_ParamUpdateConnHandle = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;
		...

}
```

---
## 关于新增加的api的使用
	1. 获取当前的连接参数
```c
{
    //check current link statsus
    uint8 linkNum;
    uint16 connIntv;
    uint16 connLatency;
    uint16 connTimeout;
    uint16 connId[MAX_NUM_LL_CONN];
    uint8_t peerAddr[6];

    GAPRole_GetParameter(GAPROLE_CONNECTION_ACTIVE_NUM,&linkNum);            
    GAPRole_GetParameter(GAPROLE_CONNHANDLE,connId);
    LOG("[LINK NUM] %d \n",linkNum);

    for(uint8_t i=0;i<linkNum;i++)
    {
        if(connId[i]!=INVALID_CONNHANDLE)
        {
            GAPRole_GetParameter_MultiConn(connId[i],GAPROLE_CONN_INTERVAL,&connIntv);  
            GAPRole_GetParameter_MultiConn(connId[i],GAPROLE_CONN_LATENCY,&connLatency);   
            GAPRole_GetParameter_MultiConn(connId[i],GAPROLE_CONN_TIMEOUT,&connTimeout);   
            GAPRole_GetParameter_MultiConn(connId[i],GAPROLE_CONN_BD_ADDR,peerAddr); 
            LOG("[%d] ConnId %d I%04d L%04d T%05d MAC:",i,connId[i],connIntv,connLatency,connTimeout);
            LOG_DUMP_BYTE(peerAddr, 6);
        }
    }
}
```
	2. 更新对应link的连接参数
```c
           
    //step 1 config conn para
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &esired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    //step 2 config conn handle
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_CONN_HANDLE, sizeof( uint16 ), &updateConnHandler );
    //step 3 send conn update req
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &updateConnParams );
           
```

	3. 对应link的notify 配置
```c
bStatus_t simpleProfile_Notify(uint16 connHandle, uint8 param, uint8 len, void* value )
{
	...
//查询对应的有效link
	for (int i=0;i<MAX_NUM_LL_CONN;i++)
	{
		if(connHandle==simpleProfileChar6Config[i].connHandle)
		{
			notfEnable = GATTServApp_ReadCharCfg( i, simpleProfileChar6Config );
			
			notfConnId = i;
			break;
		}
		
	}
	//g
	if(notfConnId==INVALID_CONNHANDLE)
	{
		ret = INVALIDPARAMETER;
	}
	// If notifications enabled
	else if ( notfEnable & GATT_CLIENT_CFG_NOTIFY )
	{
		VOID osal_memcpy( simpleProfileChar6, value, len );
		//for SAR test copy the seqNum in pkt tail
		simpleProfileChar6[ATT_GetCurrentMTUSize(0)-4-1]=simpleProfileChar6[0];
		simpleProfileChar6[ATT_GetCurrentMTUSize(0)-3-1]=simpleProfileChar6[1];
		ret=GATTServApp_ProcessCharCfg( &simpleProfileChar6Config[notfConnId], simpleProfileChar6, FALSE,
										simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
										INVALID_TASK_ID );
	}
	else
	{
		ret = bleNotReady;
	}

}
```


---
## 连接广播间隔配置
	1. TGAP_CONN_ADV_INT_MIN ，TGAP_CONN_ADV_INT_MAX
```c
  {
        uint16 advInt = 160;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );

        GAP_SetParamValue( TGAP_CONN_ADV_INT_MIN, advInt ); //连接时候的adv 间隔需要配置
        GAP_SetParamValue( TGAP_CONN_ADV_INT_MAX, advInt ); //连接时候的adv 间隔需要配置
    }
```


###
