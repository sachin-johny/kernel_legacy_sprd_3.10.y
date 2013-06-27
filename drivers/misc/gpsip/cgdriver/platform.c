/******************************************************************************
 *                                LEGAL NOTICE                                *
 *                                                                            *
 *  USE OF THIS SOFTWARE (including any copy or compiled version thereof) AND *
 *  DOCUMENTATION IS SUBJECT TO THE SOFTWARE LICENSE AND RESTRICTIONS AND THE *
 *  WARRANTY DISLCAIMER SET FORTH IN LEGAL_NOTICE.TXT FILE. IF YOU DO NOT     *
 *  FULLY ACCEPT THE TERMS, YOU MAY NOT INSTALL OR OTHERWISE USE THE SOFTWARE *
 *  OR DOCUMENTATION.                                                         *
 *  NOTWITHSTANDING ANYTHING TO THE CONTRARY IN THIS NOTICE, INSTALLING OR    *
 *  OTHERISE USING THE SOFTWARE OR DOCUMENTATION INDICATES YOUR ACCEPTANCE OF *
 *  THE LICENSE TERMS AS STATED.                                              *
 *                                                                            *
 ******************************************************************************/
/* Version: 1.8.9\3686 */
/* Build  : 13 */
/* Date   : 12/08/2012 */

/**
	\file
	\brief platform/CPU specific code

	\defgroup CGX_DRIVER_PLATFORM_ANDROID CPU/Android platform specific defines
	\{
*/

#include <asm/uaccess.h>
//#include <bsp.h>	// TODO - if this file is needed, compile kernel accrodingly, and un-remark
//bxd
//#include <mach/irqs.h>


#include "CgxDriverApi.h"
#include "CgCpu.h"
#include "platform.h"
#include "CgReturnCodes.h"
#include "CgxDriverOs.h"
#include "CgxDriverCore.h"




/** Native byte order for this CPU */
const TCgByteOrder CGX_DRIVER_NATIVE_BYTE_ORDER = ECG_BYTE_ORDER_4321;	// SPRD_FPGA is little endian

/*
    Reset the core and timers and initialize GPIO (direction and special function)
*/
TCgReturnCode CgxDriverInit(void *pDriver)
{
	// Check all addresses are defined
	/*
	#if CG_DRIVER_CGSNAP_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_CGSNAP_BASE_PA with correct address
	#endif
	#if CG_DRIVER_REVISION_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_REVISION_BASE_PA with correct address
	#endif
	#if CG_DRIVER_GPIO_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_GPIO_BASE_PA with correct address
	#endif
	#if CG_DRIVER_DMA_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_DMA_BASE_PA with correct address
	#endif
	#if CG_DRIVER_INTR_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_INTR_BASE_PA with correct address
	#endif
	#if CG_DRIVER_CLK_BASE_PA == CG_UNDEFINED_ADDRESS
		#error please define CG_DRIVER_CLK_BASE_PA with correct address
	#endif
	*/
	// TODO 1st time initialization

    return ECgOk;
}






TCgReturnCode CgxDriverDataReadyPrepare(void)
{
	return CgCpuGpioModeSet(CGX5000_DR, ECG_CPU_GPIO_FALLING_INT);
}


TCgReturnCode CgxDriverInterruptPrepare(void)
{
	TCgReturnCode rc = ECgOk;

	// not using 'GPS' interrupt, so there is nothing to do

	return rc;
}

TCgReturnCode CgxDriverDmaInterruptPrepare(void)
{
	// TODO initial setup for DMA interrupt.
	// if no special work is required, leave this function empty
	return ECgOk;
}




TCgReturnCode CgxDriverGpsInterruptPrepare(void)
{
	// TODO return the interrupt code for 'CGsnap' interrupt
	// if no special work is required, leave this function empty
	return ECgOk;
}




U32 CgxDriverDataReadyInterruptCode(void)
{
	// TODO return the interrupt code for 'DMA end of transfer' interrupt

	return 0;
}


U32 CgxDriverGpsInterruptCode(void)
{
	// TODO return the interrupt code for 'CGsnap' interrupt
	return IRQ_GPS_INT;
}


TCgReturnCode CgxDriverWriteData(unsigned char *aSourceVirtualAddress, unsigned long aSourcePhysicalAddress, unsigned long aLengthBytes)
{
    //DBG_FUNC_NAME("CgxDriverWriteData")
    TCgReturnCode rc = ECgOk;

// 	#if WRITE_MODE == CG_CPU_SPI_MODE_DMA
// 		if (OK(rc)) rc = CgCpuSpiDmaStop(SPI_CHANNEL);
// 		CgCpuCacheSync();
// 		CgCpuSpiSetup(SPI_CHANNEL, SPI_PRESCALE, ECG_CPU_SPI_OUTPUT_DMA);
// 		CgCpuDmaSetupToDevice(aSourcePhysicalAddress,aLengthBytes,0,0);
// 		if (OK(rc)) rc = CgCpuSpiDmaStart(SPI_CHANNEL);
// 		if (OK(rc)) rc = CgCpuDmaWaitFinish(CG_DRIVER_DMA_CHANNEL_WRITE);
// 	#elif WRITE_MODE == CG_CPU_SPI_MODE_MANUAL
// 		U32 i;
// 		CgCpuSpiSetup(SPI_CHANNEL, SPI_PRESCALE, ECG_CPU_SPI_OUTPUT_POLLING);
//
// 		for(i = 0; i < aLengthBytes; i++) {
// 			CgCpuSpiTxFinishWait(SPI_CHANNEL);
// 			CgCpuSpiWriteByte(SPI_CHANNEL, aSourceVirtualAddress[i]);
// 		}
// 	#endif

    return rc;
}


TCgReturnCode CgxDriverGpioCode(int aControlLine, int *aGpioCode)
{
	*aGpioCode = 0;
	return ECgNotSupported;
}

TCgReturnCode CgxDriverReadData(unsigned char *aTargetVirtualAddress, unsigned long aTargetPhysicalAddress, unsigned long aLengthBytes)
{
	//DBG_FUNC_NAME("CgxDriverReadData")
	TCgReturnCode rc = ECgOk;

// 	#if READ_MODE==CG_CPU_SPI_MODE_DMA
// 		// memset(aTargetVirtualAddress, 0x55, aLengthBytes); //NG! debug
// 		CgCpuCacheSync();//NG! debug
// 		// Buffer data (not register data) is written in DMA mode
// 		if (OK(rc)) rc = CgCpuSpiDmaStop(SPI_CHANNEL);
//
// 		CgCpuCacheSync();
// 		if (OK(rc)) rc = CgCpuSpiSetup(SPI_CHANNEL, SPI_PRESCALE, ECG_CPU_SPI_INPUT_DMA);
// 		if (OK(rc)) rc = CgCpuDmaSetupFromDevice(aTargetPhysicalAddress, aLengthBytes, SPI_PRESCALE, CG_DRIVER_DMA_CHANNEL_READ, CG_DRIVER_DMA_CHANNEL_WRITE, SPI_CHANNEL);
// 		if (OK(rc)) rc = CgCpuSpiDmaStart(SPI_CHANNEL);
// 		// no need to wait for DMA to finish
//
// 	#elif READ_MODE==CG_CPU_SPI_MODE_MANUAL
// 		unsigned long i;
// 		CgCpuSpiSetup(SPI_CHANNEL, SPI_PRESCALE, ECG_CPU_SPI_INPUT_POLLING);
// 		for(i = 0; i < aLengthBytes; i++) {
// 			CgCpuSpiWriteByte(SPI_CHANNEL, 0); //dummy transmit
// 			CgCpuSpiRxFinishWait(SPI_CHANNEL);
// 			CgCpuSpiReadByte(SPI_CHANNEL, (volatile unsigned char *)(aTargetVirtualAddress + i));
// 			}
// 	#endif

	return rc;

}


TCgReturnCode CgxDriverClearData(unsigned char *aTargetVirtualAddress, unsigned long aTargetPhysicalAddress, unsigned long aLengthBytes)
{
	return CgxDriverReadData(aTargetVirtualAddress, aTargetPhysicalAddress, aLengthBytes);
}


TCgReturnCode CgxDriverIsDataReady(void)
{
	unsigned int val = 0;
	CgCpuGpioModeSet(CGX5000_DR, ECG_CPU_GPIO_INPUT);
	CgCpuGpioGet(CGX5000_DR, &val);
	return (val == 0) ? ECgOk : ECgGeneralFailure;
}


TCgReturnCode CgxDriverReadDataTail(unsigned char *aTargetVirtualAddress, unsigned long aTargetPhysicalAddress, unsigned long aLengthBytes)
{
	// no tail reading is required for SPRD_FPGA

	return ECgOk;
}

TCgReturnCode CgxDriverRFPowerDown(void)
{
	//DBG_FUNC_NAME("CgxDriverRFPowerDown")
	TCgReturnCode rc = ECgOk;
	int value;
	// TODO : implement

	/*RF4in1 0X0700*/
	value = 0x07000000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);

	return rc;
}

TCgReturnCode CgxDriverRFPowerUp(void)
{
	//DBG_FUNC_NAME("CgxDriverRFPowerUp")
	TCgReturnCode rc = ECgOk;
	int value;
	// TODO : implement
	//bxd add for RF config

	CgxCpuReadMemory((U32)CG_RF_ARM_BASE_VA, 0X0038, (U32 *)&value);
	value = (1<<4)|value;
	value = (0<<5)|value;
	CgxCpuWriteMemory((U32)CG_RF_ARM_BASE_VA, 0X0038,value);


	CgxCpuReadMemory((U32)CG_RF_ARM_BASE_VA, 0X003C, (U32 *)&value);
	value = (1<<4)|value;
	value = (0<<5)|value;
	CgxCpuWriteMemory((U32)CG_RF_ARM_BASE_VA, 0X003C,value);


	CgxCpuReadMemory((U32)CG_RF_ARM_BASE_VA, 0X0040, (U32 *)&value);
	value = (1<<4)|value;
	value = (0<<5)|value;
	CgxCpuWriteMemory((U32)CG_RF_ARM_BASE_VA, 0X0040,value);


	CgxCpuReadMemory((U32)CG_RF_APB_EB0_BASE_VA, 0x0, (U32 *)&value);
	value = (1<<23)|value;
	CgxCpuWriteMemory((U32)CG_RF_APB_EB0_BASE_VA, 0x0,value);


	CgxCpuReadMemory((U32)CG_RF_ARM_BASE_VA, 0x0, (U32 *)&value);
	value = (1<<26)|value;
	value = (1<<20)|value;
	CgxCpuWriteMemory((U32)CG_RF_ARM_BASE_VA, 0x0,value);



	/*RF4in1 0X04a*/
	value = 0x004af417;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x804a0000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X0738*/
	value = 0x07385400;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x87380000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X076c*/
	value = 0x076c0721;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x876c0000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X0763*/
	value = 0x07635141;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x87630000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X0704*/
	value = 0x0704ef00;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x87040000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X0700*/
	value = 0x07000001;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x87000000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	/*RF4in1 0X073c*/
	value = 0x073c08da;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x000c,value);
	value = 0x873c0000;
	CgxCpuWriteMemory((U32)CG_RF_MSPI_BASE_VA, 0x0010,value);


	//CgxCpuReadMemory(CG_DRIVER_SCLK_VA, 0x0, &value);
	//value = (1<<5)|value;
	//value = (1<<12)|value;
	//CgxCpuWriteMemory(CG_DRIVER_SCLK_VA, 0x0,value);

	return rc;
}


TCgReturnCode CgxDriverTcxoEnable(U32 aEnable)
{
	// default implementation assumes direct manipulation of TCXO LDO via a dedicated GPIO pin.
	// either CG_DRIVER_GPIO_TCXO_ENABLE or CG_DRIVER_GPIO_TCXO_DISABLE should be defined depending
	// on the LDO control level (active high / active low)

	#if defined(CG_DRIVER_GPIO_TCXO_ENABLE)
		if (aEnable)
			CgCpuGpioSet(CG_DRIVER_GPIO_TCXO_ENABLE);
		else
			CgCpuGpioReset(CG_DRIVER_GPIO_TCXO_ENABLE);
	#elif defined(CG_DRIVER_GPIO_TCXO_DISABLE)
		if (aEnable)
			CgCpuGpioReset(CG_DRIVER_GPIO_TCXO_DISABLE);
		else
			CgCpuGpioSet(CG_DRIVER_GPIO_TCXO_DISABLE);
	#endif

	return ECgOk;
}



TCgReturnCode CgCpuRevision(char *apRevisionCode)
{

    //sprintf(apRevisionCode,"SPRD_FPGA-R255",0);
    sprintf(apRevisionCode,"SPRD_FPGA-R255");

	return ECgOk;

}


TCgReturnCode CgCpuClockDestroy(void)
{
	TCgReturnCode rc = ECgOk;
	return rc;
}

TCgReturnCode CgCpuIntrDestroy(void)
{
 	TCgReturnCode rc = ECgOk;
 //   #if defined(CG_DRIVER_INTR_BASE_PA) && !defined(CG_DRIVER_INTR_BASE_VA)
//        rc = CgCpuVirtualFree((volatile void **)&gpINTR);
 //   #endif
	return rc;
}


TCgReturnCode CgCpuIPMasterResetClear(void)
{
	TCgReturnCode rc = ECgOk;
        DBG_FUNC_NAME("CgCpuIPMasterResetClear");

	DBGMSG("start");
	// Set master reset pin to '1' (reset off - device active)
   // CgCpuGpioSet(CG_DRIVER_GPIO_GPS_MRSTN);

	return rc;
}




TCgReturnCode CgCpuIPMasterResetOn(void)
{
	TCgReturnCode rc = ECgOk;
    DBG_FUNC_NAME("CgCpuIPMasterResetOn");

	DBGMSG("start");
	// Set master reset pin to '0' (reset on - device inactive)

    CgCpuGpioReset(CG_DRIVER_GPIO_GPS_MRSTN);

	return rc;
}


TCgReturnCode CgCpuAllocateVa(void)
{
	// Nothing to do for SPRD_FPGA
	return ECgOk;
}











/**
	\}
*/

