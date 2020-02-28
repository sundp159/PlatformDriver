#include <c6x.h>
#include <csl_cacheAux.h>
#include <string.h>
#include "MulticoreSync.h"
#include "PlatformNto1.h"


void iUpdateSyncStatus(structMultiCoreSetting mcs){
	if(mcs.CurrentCore == mcs.MasterCore)//MPP_MASTER_CORE)
	{
		if(SyncStatus.ErrorStatus != 0){
			return;
		}
		int mpptime = mcs.MPPTime;
		int m_i = 0;
		int m_tempCount = 0;
		int m_startIndex = mcs.MasterCore + 1;//MPP_MASTER_CORE + 1;
		while( (m_tempCount != mcs.MultiCoreCount - 1) && (mpptime > 0))//MPP_CORE_NUM - 1)
		{
			//判断11--17核更新状态是否为0
			for(m_i = m_startIndex, m_tempCount = 0; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
				if(SyncStatus.IsUpdate[m_i] == MPP_STATUS_NOTUPDATE){
					m_tempCount++;
				}
			}
			delay_us(MPP_TIMEOUTSTEP_US);
			mpptime = mpptime - MPP_TIMEOUTSTEP_US;
		}

		if(mpptime <= 0){//20160629
			SyncStatus.ErrorStatus = 1;
			return;
		}
		//将11--17核的更新状态设置为已更新状态
		for( m_i = m_startIndex; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++ ){
			SyncStatus.IsUpdate[m_i] = MPP_STATUS_UPDATED;
		}
		m_tempCount = 0;
		mpptime = mcs.MPPTime;
		//等待11--17核的更新状态被置为0
		while( (m_tempCount != mcs.MultiCoreCount - 1) && (mpptime > 0)){
			for(m_i = m_startIndex, m_tempCount = 0; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
				if(SyncStatus.IsUpdate[m_i] == MPP_STATUS_NOTUPDATE){
					m_tempCount++;
				}
			}
			delay_us(MPP_TIMEOUTSTEP_US);
			mpptime = mpptime - MPP_TIMEOUTSTEP_US;
		}
		if(mpptime <= 0){//20160629
			SyncStatus.ErrorStatus = 1;
			return;
		}
	}
	else
	{
		//等待11--17核更新标志被置为已更新状态
		while( SyncStatus.IsUpdate[mcs.CurrentCore] != MPP_STATUS_UPDATED ){delay_us(MPP_TIMEOUTSTEP_US);}//从片死等
		//备份多核同步运行状态为当前更新状态
		MultiCoreSync[mcs.CurrentCore].SyncStatusCopy = SyncStatus;
		//设置多核同步运行标志当前更新状态为0
		SyncStatus.IsUpdate[mcs.CurrentCore] = MPP_STATUS_NOTUPDATE;
	}
}





void InitParralleProcess(structMultiCoreSetting mcs){
	if(mcs.CurrentCore == mcs.MasterCore)
	{
//		if(SyncStatus.ErrorStatus != 0){
//			return;
//		}
		//mcs.MPPTime = 1000 * 1000;
		int mpptime = mcs.MPPTime;
		//初始化多核同步运行标志
		memset( &MultiCoreSync, 0, sizeof(MultiCoreSync) );
		//初始化同步运行状态结构体
		memset( &SyncStatus, 0, sizeof(SyncStatus) );
		//设置以前同步状态为无状态
		SyncStatus.PreviousStatus = MPP_STATUS_NONE;
		//设置当前同步状态为已准备状态
		SyncStatus.CurrentStatus = MPP_STATUS_READY;
		//设置同步状态嵌套数为0
		SyncStatus.NestedCount = 0;
		//更新同步运行状态
		iUpdateSyncStatus(mcs);
	
		int m_i = 0;
		int m_tempCount = 0;
		int m_startIndex = mcs.MasterCore + 1;
		while( (m_tempCount != mcs.MultiCoreCount - 1) && (mpptime > 0)){
			for(m_i = m_startIndex, m_tempCount = 0; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
				//判断11--17核的同步运行标志是否为结束标志
				if(MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] == MPP_FLAG_STOP){//MPP_FLAG_NONE
					m_tempCount++;
				}
			}
			delay_us(MPP_TIMEOUTSTEP_US);
			mpptime = mpptime - MPP_TIMEOUTSTEP_US;
		}
		if(mpptime <= 0){
			SyncStatus.ErrorStatus = 1;
			//MultiCoreSync[mcs.MasterCore].SyncStatusCopy.ErrorStatus = 1;
			return;
		}
		//设置11--17核同步运行状态为结束
		for(m_i = m_startIndex; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
			MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] = MPP_FLAG_STOP;
		}
	}
	else{
		iUpdateSyncStatus(mcs);
		
		//while( MultiCoreSync[mcs.CurrentCore].SyncStatusCopy.CurrentStatus != MPP_STATUS_READY ){TSC_delay_us(10);}
		//获取同步运行嵌套数目
		int s_nestedCount = MultiCoreSync[mcs.CurrentCore].SyncStatusCopy.NestedCount;
		//等待11--17核同步运行标志被置为结束标志
		while( MultiCoreSync[mcs.CurrentCore].SyncFlag[s_nestedCount] != MPP_FLAG_STOP){delay_us(MPP_TIMEOUTSTEP_US);}
	}
}





void StartParrallelProcess(structMultiCoreSetting mcs){
	if(mcs.CurrentCore == mcs.MasterCore)
	{
		if(SyncStatus.ErrorStatus != 0){
			return;
		}
		//设置缓存中的所有脏数据回写
		CACHE_wbInvAllL1d(CACHE_FENCE_WAIT);
		//判断当前运行状态是否是已准备状态
		if(SyncStatus.CurrentStatus == MPP_STATUS_READY)
		{
			//设置同步运行状态嵌套数目为0
			SyncStatus.NestedCount = 0;
		}
		//判断当前运行状态是否为开始状态
		if(SyncStatus.CurrentStatus == MPP_STATUS_START)
		{
			//设置运行状态嵌套数目自加
			SyncStatus.NestedCount++;
		}
		//设置以前的同步运行状态为当前同步运行状态
		SyncStatus.PreviousStatus = SyncStatus.CurrentStatus;
		//更新当前同步运行状态为开始状态
		SyncStatus.CurrentStatus = MPP_STATUS_START;

		iUpdateSyncStatus(mcs);
		//mcs.MPPTime = 200 * 1000;
		int mpptime = mcs.MPPTime;
		int m_i = 0;
		int m_tempCount = 0;
		int m_startIndex = mcs.MasterCore + 1;
		while( (m_tempCount != mcs.MultiCoreCount - 1) && (mpptime > 0))
		{
			for(m_i = m_startIndex, m_tempCount = 0; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
				//判断11--17核的同步运行标志是否为结束标志
				if(MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] == MPP_FLAG_STOP){
					m_tempCount++;
				}
			}
			delay_us(MPP_TIMEOUTSTEP_US);
			mpptime = mpptime - MPP_TIMEOUTSTEP_US;
		}

		if(mpptime <= 0){
			SyncStatus.ErrorStatus = 1;
			//MultiCoreSync[mcs.MasterCore].SyncStatusCopy.ErrorStatus = 1;
			return;
		}

		//设置11--17核同步运行状态为开始
		for(m_i = m_startIndex; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
			MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] = MPP_FLAG_START;
		}
//		iUpdateSyncStatus();
	}
	else
	{
		iUpdateSyncStatus(mcs);
		//获取同步运行嵌套数目
		int s_nestedCount = MultiCoreSync[mcs.CurrentCore].SyncStatusCopy.NestedCount;
		//等待11--17核同步运行标志被置为结束标志
		while( MultiCoreSync[mcs.CurrentCore].SyncFlag[s_nestedCount] != MPP_FLAG_START){delay_us(MPP_TIMEOUTSTEP_US);}
	}
}





void StopParrallelProcess(structMultiCoreSetting mcs){
	CACHE_wbInvAllL1d(CACHE_FENCE_WAIT);

	if(mcs.CurrentCore == mcs.MasterCore){
		if(SyncStatus.ErrorStatus != 0){
			return;
		}
		//判断当前同步运行状态是否为已结束状态
		if(SyncStatus.CurrentStatus == MPP_STATUS_STOP)
		{
			//设置同步运行状态嵌套数目自减
			SyncStatus.NestedCount--;
		}
		//设置以前的同步运行状态为当前同步运行状态
		SyncStatus.PreviousStatus = SyncStatus.CurrentStatus;
		//更新当前同步运行状态为结束状态
		SyncStatus.CurrentStatus = MPP_STATUS_STOP;
		iUpdateSyncStatus(mcs);
		//mcs.MPPTime = 200 * 1000;
		int mpptime = mcs.MPPTime;
		int m_i = 0;
		int m_tempCount = 0;
		int m_startIndex = mcs.MasterCore + 1;
		while( (m_tempCount != mcs.MultiCoreCount - 1) && (mpptime > 0)){
			for(m_i = m_startIndex, m_tempCount = 0; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
				//等待11--17核的同步运行标志被置为等待标志
				if(MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] == MPP_FLAG_WAIT){
					m_tempCount++;
				}
			}
			delay_us(MPP_TIMEOUTSTEP_US);
			mpptime = mpptime - MPP_TIMEOUTSTEP_US;
		}

		if(mpptime <= 0){
			SyncStatus.ErrorStatus = 1;
			//MultiCoreSync[mcs.MasterCore].SyncStatusCopy.ErrorStatus = 1;
			return;
		}
		//设置11--17核的同步运行标志为结束标志
		for(m_i = m_startIndex; m_i < (mcs.MasterCore + mcs.MultiCoreCount); m_i++){
			MultiCoreSync[m_i].SyncFlag[SyncStatus.NestedCount] = MPP_FLAG_STOP;
		}

//		CACHE_wbInvAllL1d(CACHE_FENCE_WAIT);
	}
	else
	{
		//更新同步运行状态
		iUpdateSyncStatus(mcs);
		//获取同步运行嵌套数目
		int s_nestedCount = MultiCoreSync[mcs.CurrentCore].SyncStatusCopy.NestedCount;
		MultiCoreSync[mcs.CurrentCore].SyncFlag[s_nestedCount] = MPP_FLAG_WAIT;
		//待11--17核的同步运行标志被置为结束标志
		while(MultiCoreSync[mcs.CurrentCore].SyncFlag[s_nestedCount] !=  MPP_FLAG_STOP){delay_us(MPP_TIMEOUTSTEP_US);}
//		CACHE_wbInvAllL1d(CACHE_FENCE_WAIT);
	}
}


