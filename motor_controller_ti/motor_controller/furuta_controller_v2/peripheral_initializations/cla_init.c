// Includes
#include "cla_init.h"

#include "cpu_cla_shared.h"
#include "driverlib.h"
#include "device.h"

//
// CLA_configClaMemory - Configure CLA memory sections
//
void cla_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU)){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
    while (!MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1)){};

    //
    // Select LS4RAM and LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS4 and LS5 and then
    // set the space to be a program block
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4,MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5,MEMCFG_CLA_MEM_PROGRAM);
    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1,MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//
void cla_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);

    DEVICE_DELAY_US(1);

    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_1,(uint16_t)&motor_torque_control);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_2,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_3,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_4,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_5,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_6,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_7,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_8,(uint16_t)&sw_task);

    DEVICE_DELAY_US(1);

    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_ALL);

    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_8);

    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_ADCA1); // CLA_TRIGGER_ADCA1 implies ADCA Interrupt 1, and not ADCA Channel 1

    EDIS;

}

