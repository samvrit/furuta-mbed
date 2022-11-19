// Included Files
#include <cpu_cla_shared.h>
#include "driverlib.h"
#include "device.h"

// Defines
#define WAITSTEP     asm(" RPT #255 || NOP")

// Globals


//
//Task 1 (C) Variables
// NOTE: Do not initialize the Message RAM variables globally, they will be
// reset during the message ram initialization phase in the CLA memory
// configuration routine
//
#pragma DATA_SECTION(cla_inputs,"CpuToCla1MsgRAM");
struct cla_inputs_S cla_inputs;

#pragma DATA_SECTION(cla_outputs,"Cla1ToCpuMsgRAM");
struct cla_outputs_S cla_outputs;

#pragma DATA_SECTION(cpu_cla_shared,"CLADataLS0")
struct cpu_cla_shared_S cpu_cla_shared;


//
// Function Prototypes
//
void CLA_runTest(void);
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);
__interrupt void cla1Isr1();

//
// Main
//
void main(void)
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();

    // Configure the CLA memory spaces first followed by
    // the CLA task vectors
    CLA_configClaMemory();
    CLA_initCpu1Cla1();

    // Enable Global Interrupts (INTM) and realtime interrupt (DGBM)

    EINT;
    ERTM;

    //
    // Run the test
    //
    CLA_runTest();

    for(;;)
    {
    }
}

//
// CLA_runTest - Execute CLA task tests for specified vectors
//
void CLA_runTest(void)
{

}

//
// CLA_configClaMemory - Configure CLA memory sections
//
void CLA_configClaMemory(void)
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
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_1,(uint16_t)&motor_torque_control);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_2,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_3,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_4,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_5,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_6,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_7,(uint16_t)NULL);
    CLA_mapTaskVector(CLA1_BASE,CLA_MVECT_8,(uint16_t)NULL);
    EDIS;

}

//
// End of file
//
