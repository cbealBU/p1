/* canac2pcisetup.c - xPC Target, non-inlined S-function driver for CAN-AC2-PCI from Softing (single board)  */
/* Copyright 1996-2009 The MathWorks, Inc.
*/

#define DEBUG 0


#define 	S_FUNCTION_LEVEL 	2
#undef 		S_FUNCTION_NAME
#define 	S_FUNCTION_NAME 	canac2104setup

#include 	<stddef.h>
#include 	<stdlib.h>

#ifndef 	MATLAB_MEX_FILE
#undef 		WIN32
#define 	WIN32
#include 	"can_def.h"
#include 	"canlay2_mb1.h" 
#include 	"canlay2_mb2.h"
#include 	"canlay2_mb3.h"
#undef 		WIN32
#endif

#include 	"tmwtypes.h"
#include 	"simstruc.h" 

#ifdef 		MATLAB_MEX_FILE
#include 	"mex.h"
#else
#include 	<windows.h>
#include 	"xpcimports.h"
#endif

#ifndef     MATLAB_MEX_FILE
#include    "canac2globals.h"
#endif

#define 	NUMBER_OF_ARGS          	(22)
#define 	CAN1_USER_BAUDRATE_ARG      ssGetSFcnParam(S,0)
#define 	CAN2_USER_BAUDRATE_ARG      ssGetSFcnParam(S,1)
#define 	CAN1_SEND_ARG               ssGetSFcnParam(S,2)
#define 	CAN1_RECEIVE_ARG            ssGetSFcnParam(S,3)
#define 	CAN2_SEND_ARG               ssGetSFcnParam(S,4)
#define 	CAN2_RECEIVE_ARG            ssGetSFcnParam(S,5)
#define 	CAN1_SENDE_ARG              ssGetSFcnParam(S,6)
#define 	CAN1_RECEIVEE_ARG           ssGetSFcnParam(S,7)
#define 	CAN2_SENDE_ARG              ssGetSFcnParam(S,8)
#define 	CAN2_RECEIVEE_ARG           ssGetSFcnParam(S,9)
#define 	CAN1_INTS_ARG           	ssGetSFcnParam(S,10)
#define 	CAN1_INTE_ARG           	ssGetSFcnParam(S,11)
#define 	CAN2_INTS_ARG           	ssGetSFcnParam(S,12)
#define 	CAN2_INTE_ARG           	ssGetSFcnParam(S,13)
#define 	CAN_INIT_ARG           		ssGetSFcnParam(S,14)
#define 	CAN_TERM_ARG           		ssGetSFcnParam(S,15)
#define 	IO_BASE_ADDRESS_ARG         ssGetSFcnParam(S,16)
#define 	MEM_BASE_ADDRESS_ARG        ssGetSFcnParam(S,17)
#define 	INTNO_ARG        			ssGetSFcnParam(S,18)
#define 	BOARD_ARG        			ssGetSFcnParam(S,19)
#define     BUS_OFF_OUT_ARG             ssGetSFcnParam(S,20)
#define     BUS_OFF_REC_ARG             ssGetSFcnParam(S,21)

#define 	NO_I_WORKS                  (0)
#define 	NO_R_WORKS                  (0)

#define 	MAX_OBJECTS 				200
#define 	MAX_IDS						2031
#define 	MAX_IDS_EXT  				536870911

// Acceptance filter
#define		ACCEPT_MASK_1				0x0000
#define		ACCEPT_CODE_1 				0x0000
#define		ACCEPT_MASK_XTD_1			0x00000000L
#define		ACCEPT_CODE_XTD_1			0x00000000L

#define		ACCEPT_MASK_2				0x0000
#define		ACCEPT_CODE_2 				0x0000
#define		ACCEPT_MASK_XTD_2			0x00000000L
#define		ACCEPT_CODE_XTD_2			0x00000000L
                                           

// Resynchronization
#define		SLEEPMODE_1					0
#define		SPEEDMODE_1					0
                                           
#define		SLEEPMODE_2					0
#define		SPEEDMODE_2					0


// Phys. layer (-1: default - CAN High Speed)
#define		OUTPUT_CONTROL_1			-1                                           
#define		OUTPUT_CONTROL_2			-1  

#define		CAN_RECOVERY_DELAY          750

static char_T msg[256];

#ifndef MATLAB_MEX_FILE

static  HANDLE can_recovery_thread;
static  HANDLE bus_status_semaphore;
static  char sem_name[100];
static int CANAC2_FIRMWARE_LOADED[3]={0};
static DWORD WINAPI canRecoveryFunc( LPVOID lpParam ); 

#if 0  // Not used, but keep it around for reference, may need it later
// Routine to evaluate error codes of CANPC_initialize_board
static void printInitErrorText(int iErrorCode)
{
    printf("ErrorCode=%x  Error Text:", iErrorCode);
    switch((unsigned int)iErrorCode)
    {
      case  INIPC_IB_PNP_NO_DEVICE_FOUND:
         printf("no can device found ");
         break;
     case  INIPC_IB_ERR_VC_INTERNALERROR:
         printf("internal error ");
         break;
     case  INIPC_IB_ERR_VC_GENERALERROR:
         printf("general error ");
         break;
     case  INIPC_IB_ERR_VC_TIMEOUT:
         printf("Timeout ");
         break;
     case  INIPC_IB_ERR_VC_IOPENDING:
         printf("driver call pending ");
         break;
     case  INIPC_IB_ERR_VC_IOCANCELLED:
         printf("driver call cancelled ");
         break;
     case  INIPC_IB_ERR_VC_ILLEGALCALL:
         printf("illegal driver call ");
         break;
     case  INIPC_IB_ERR_VC_NOTSUPPORTED:
         printf("driver call not supported ");
         break;
     case  INIPC_IB_ERR_VC_VERSIONERROR:
         printf("wrong driver-dll version ");
         break;
     case  INIPC_IB_ERR_VC_DRIVERVERSIONERROR:
         printf("wrong driver version ");
         break;
     case  INIPC_IB_ERR_VC_DRIVERNOTFOUND:
         printf("driver not found ");
         break;
     case  INIPC_IB_ERR_VC_NOTENOUGHMEMORY:
         printf("not enough memory ");
         break;
     case  INIPC_IB_ERR_VC_TOOMANYDEVICES:
         printf("too many devices ");
         break;
     case  INIPC_IB_ERR_VC_UNKNOWNDEVICE:
         printf("unknown device ");
         break;
     case  INIPC_IB_ERR_VC_DEVICEALREADYEXISTS:
         printf("Device ardy exists ");
         break;
     case  INIPC_IB_ERR_VC_DEVICEACCESSERROR:
         printf("device ardy open ");
         break;
     case  INIPC_IB_ERR_VC_RESOURCEALREADYREGISTERED:
         printf("Resource in use");
         break;
     case  INIPC_IB_ERR_VC_RESOURCECONFLICT:
         printf("Resource-conflict ");
         break;
     case  INIPC_IB_ERR_VC_RESOURCEACCESSERROR:
         printf("Resource access error");
         break;
     case  INIPC_IB_ERR_VC_PHYSMEMORYOVERRUN:
         printf("invalid phys.mem-access");
         break;
     case  INIPC_IB_ERR_VC_TOOMANYPORTS:
         printf("too many I/O ports ");
         break;
     case  INIPC_IB_ERR_VC_UNKNOWNRESOURCE:
         printf("unknown resource ");
         break;
     default:
         printf("unknown return value !");
         break;
     }
     printf ("\n");
}
#endif

// Routine to evaluate error codes of CANPC_reset_board
static void printResetErrorText(int iErrorCode)
{
    printf("ErrorCode=%d  Error Text:", iErrorCode);
    switch(iErrorCode)
    {
     case CANPC_RB_INI_FILE:
          printf(" can't open IniFile       ");              
         break;
     case CANPC_RB_ERR_FMT_INI:
          printf(" format error in INI-file ");    
         break;
     case CANPC_RB_ERR_OP_BIN:
          printf(" error opening binary-file");    
         break;
     case CANPC_RB_ERR_RD_BIN: 
          printf(" error reading binary-file");    
         break;
     case CANPC_RB_BIN_TOO_LONG:
          printf(" binary-file too long     ");    
         break;
     case CANPC_RB_ERR_BIN_FMT: 
          printf(" binary-data format error ");    
         break;
     case CANPC_RB_ERR_BIN_CS:
          printf(" binary-data checksum error      ");   
         break;
     case CANPC_RB_NO_CARD:  
          printf(" no card present          ");    
         break;
     case CANPC_RB_NO_PHYS_MEM:
          printf(" no physical memory       ");    
         break;
     case CANPC_RB_INVLD_IRQ:  
          printf(" invalid IRQ-number       ");    
         break;
     case CANPC_RB_ERR_DPRAM_ACCESS:
          printf(" error accessing dpram    ");    
         break;
     case CANPC_RB_ERR_CRD_RESP: 
          printf(" bad response from card   ");    
         break;
     case CANPC_RB_ERR_SRAM:
          printf(" sram seems to be damaged ");    
         break;
     case CANPC_RB_ERR_PRG:
          printf(" invalid program start address   ");   
         break;
     case CANPC_RB_ERR_REC:
          printf(" invalid record type      ");    
         break;
     case CANPC_RB_ERR_NORESP:
          printf(" no response after program start ");   
         break;
     case CANPC_RB_ERR_BADRESP:
          printf(" bad response after program start");   
         break;
     case CANPC_RB_PCMCIA_NSUPP:
          printf(" pcmcia chip not supported");    
         break;
     case CANPC_RB_ERR_RD_PCMCIA:
          printf(" error reading ocmcia parameters ");   
         break;
     case CANPC_RB_INIT_CHIP:
          printf(" error initializing chip  ");    
         break;
     default:
         printf("return value ");
         break;
    }
    printf("\n");
}


static int Start_chip(SimStruct *S)
{

      switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
      case 1:
        if (CANPC_start_chip_mb1() != 0) {
            printf("-->Error in CANPC_start_chip_mb1 \n");
            return -1;
        }
        break;
      case 2:
        if (CANPC_start_chip_mb2() != 0) {
            printf("-->Error in CANPC_start_chip_mb2 \n");
            return -1;
        }
        break;
      case 3:
        if (CANPC_start_chip_mb3() != 0) {
            printf("-->Error in CANPC_start_chip_mb3 \n");
            return -1;
        }
        break;
    }
	return 0;
}
static int Initialize_DOB_mode(SimStruct *S)
{
	int_T                   msg_id;
    size_t                  i;

    int                     ReceiveFifoEnable;
    int                     ReceivePollAll;
    int                     ReceiveEnableAll;
    int                     ReceiveIntEnableAll;
    int                     AutoRemoteEnableAll;
    int                     TransmitReqFifoEnable;
    int                     TransmitPollAll;
    int                     TransmitAckEnableAll;
    int                     TransmitAckFifoEnableAll;
    int                     TransmitRmtFifoEnableAll;

    int                     Type;
    int                     ReceiveIntEnable;
    int                     AutoRemoteEnable;
    int                     TransmitAckEnable;

//    unsigned long   Identifier;

    int                     index;
//    double                  timetmp;
    int                     test;
//    int_T                   intNo;
//    int                     tmp;
    

    switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
      case 1:
        if (CANPC_enable_dyn_obj_buf_mb1() != 0) {
            printf("-->Error in CANPC_enable_dyn_obj_buf_mb1 \n");
            return -1;
        }
        break;
      case 2:
        if (CANPC_enable_dyn_obj_buf_mb2() != 0) {
            printf("-->Error in CANPC_enable_dyn_obj_buf_mb2 \n");
            return -1;
        }
        break;
      case 3:
        if (CANPC_enable_dyn_obj_buf_mb3() != 0) {
            printf("-->Error in CANPC_enable_dyn_obj_buf_mb3 \n");
            return -1;
        }
        break;
    }

    // Configuration of the object buffer
    ReceiveFifoEnable         = 0;
    ReceivePollAll            = 0;
    ReceiveEnableAll          = 0;
    ReceiveIntEnableAll       = 0;
    AutoRemoteEnableAll       = 0;
    TransmitReqFifoEnable     = 1;
    TransmitPollAll           = 0;
    TransmitAckEnableAll      = 1;
    TransmitAckFifoEnableAll  = 1;
    TransmitRmtFifoEnableAll  = 1;

    switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
      case 1:
        if (CANPC_initialize_interface_mb1(ReceiveFifoEnable,
                                           ReceivePollAll,
                                           ReceiveEnableAll,
                                           ReceiveIntEnableAll,
                                           AutoRemoteEnableAll,
                                           TransmitReqFifoEnable,
                                           TransmitPollAll,
                                           TransmitAckEnableAll,
                                           TransmitAckFifoEnableAll,
                                           TransmitRmtFifoEnableAll) != 0) {
            printf("-->Error in CANPC_initialize_interface_mb1 \n");
            return -1;
        }
        break;
      case 2:
        if (CANPC_initialize_interface_mb2(ReceiveFifoEnable,
                                           ReceivePollAll,
                                           ReceiveEnableAll,
                                           ReceiveIntEnableAll,
                                           AutoRemoteEnableAll,
                                           TransmitReqFifoEnable,
                                           TransmitPollAll,
                                           TransmitAckEnableAll,
                                           TransmitAckFifoEnableAll,
                                           TransmitRmtFifoEnableAll) != 0) {
            printf("-->Error in CANPC_initialize_interface_mb2 \n");
            return -1;
        }
        break;
      case 3:
        if (CANPC_initialize_interface_mb3(ReceiveFifoEnable,
                                           ReceivePollAll,
                                           ReceiveEnableAll,
                                           ReceiveIntEnableAll,
                                           AutoRemoteEnableAll,
                                           TransmitReqFifoEnable,
                                           TransmitPollAll,
                                           TransmitAckEnableAll,
                                           TransmitAckFifoEnableAll,
                                           TransmitRmtFifoEnableAll) != 0) {
            printf("-->Error in CANPC_initialize_interface_mb3 \n");
            return -1;
        }
        break;
    }

    // Define send and receive objects for CAN1 and CAN2

    ReceiveIntEnable  = 0;
    AutoRemoteEnable  = 1;
    TransmitAckEnable = 1; // Craig Beal changed this to 1 on 5/31/18

    test=0;
    // CAN1 send standard identifiers
    Type=1;
    for (i=0;i<mxGetN(CAN1_SEND_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN1_SEND_ARG)[i];
        ReceiveIntEnable= 0;
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object_mb1(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb1 for send objects CAN 1\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object_mb2(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb2 for send objects CAN 1\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object_mb3(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb3 for send objects CAN 1\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    // CAN1 send extended identifiers
    Type=3;
    for (i=0;i<mxGetN(CAN1_SENDE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN1_SENDE_ARG)[i];
        ReceiveIntEnable= 0;
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object_mb1(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb1 for send objects CAN 1\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object_mb2(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb2 for send objects CAN 1\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object_mb3(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb3 for send objects CAN 1\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    test=0;
    // CAN1 receive standard identifiers
    Type=0;
    for (i=0;i<mxGetN(CAN1_RECEIVE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN1_RECEIVE_ARG)[i];
        ReceiveIntEnable= (int_T)mxGetPr(CAN1_INTS_ARG)[i];
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object_mb1(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb1 for receive objects CAN 1\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object_mb2(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb2 for receive objects CAN 1\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object_mb3(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb3 for receive objects CAN 1\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    // CAN1 receive extended identifiers
    Type=2;
    for (i=0;i<mxGetN(CAN1_RECEIVEE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN1_RECEIVEE_ARG)[i];
        ReceiveIntEnable= (int_T)mxGetPr(CAN1_INTE_ARG)[i];
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object_mb1(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb1 for receive objects CAN 1\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object_mb2(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb2 for receive objects CAN 1\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object_mb3(msg_id,
                                        &index,
                                        Type,
                                        ReceiveIntEnable,
                                        AutoRemoteEnable,
                                        TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object_mb3 for receive objects CAN 1\n");
                return -1;
            }
            break;
        }
    }

    test=0;
    // CAN2 send standard identifiers
    Type=1;
    for (i=0;i<mxGetN(CAN2_SEND_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN2_SEND_ARG)[i];
        ReceiveIntEnable= 0;
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object2_mb1(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb1 for send objects CAN 2\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object2_mb2(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb2 for send objects CAN 2\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object2_mb3(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb3 for send objects CAN 2\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    // CAN2 send extended identifiers
    Type=3;
    for (i=0;i<mxGetN(CAN2_SENDE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN2_SENDE_ARG)[i];
        ReceiveIntEnable= 0;
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object2_mb1(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb1 for send objects CAN 2\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object2_mb2(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb2 for send objects CAN 2\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object2_mb3(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb3 for send objects CAN 2\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    test=0;
    // CAN2 receive standard identifiers
    Type=0;
    for (i=0;i<mxGetN(CAN2_RECEIVE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN2_RECEIVE_ARG)[i];
        ReceiveIntEnable= (int_T)mxGetPr(CAN2_INTS_ARG)[i];
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object2_mb1(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb1 for receive objects CAN 2\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object2_mb2(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb2 for receive objects CAN 2\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object2_mb3(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb3 for receive objects CAN 2\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }
    // CAN2 receive extended identifiers
    Type=2;
    for (i=0;i<mxGetN(CAN2_RECEIVEE_ARG);i++) {
        msg_id=(int_T)mxGetPr(CAN2_RECEIVEE_ARG)[i];
        ReceiveIntEnable= (int_T)mxGetPr(CAN2_INTE_ARG)[i];
        switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
          case 1:
            if (CANPC_define_object2_mb1(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb1 for receive objects CAN 2\n");
                return -1;
            }
            break;
          case 2:
            if (CANPC_define_object2_mb2(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb2 for receive objects CAN 2\n");
                return -1;
            }
            break;
          case 3:
            if (CANPC_define_object2_mb3(msg_id,
                                         &index,
                                         Type,
                                         ReceiveIntEnable,
                                         AutoRemoteEnable,
                                         TransmitAckEnable) != 0) {
                printf("-->Error in CANPC_define_object2_mb3 for receive objects CAN 2\n");
                return -1;
            }
            break;
        }
        if (index!=test) {
            ssSetErrorStatus(S,"indexing assumption failed");
            return -1;
        }
        test++;
    }    

	return 0;
}
static int  Initialize_can_parameter(SimStruct *S)
{
	int presc, sjw, tseg1, tseg2;

   	// Reset CAN chips of CAN 1 and 2

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_reset_chip_mb1() != 0) {
    		printf("-->Error in CANPC_reset_chip_mb1 \n");
      		return(-1);
   		}
		break;
	case 2:
		if (CANPC_reset_chip_mb2() != 0) {
    		printf("-->Error in CANPC_reset_chip_mb2 \n");
      		return(-1);
   		}
   		break;
	case 3:
		if (CANPC_reset_chip_mb3() != 0) {
    		printf("-->Error in CANPC_reset_chip_mb3 \n");
      		return(-1);
   		}
		break;
	}

   	// Set Bit timing for CAN 1 and 2
	presc=(int)mxGetPr(CAN1_USER_BAUDRATE_ARG)[0];
	sjw=(int)mxGetPr(CAN1_USER_BAUDRATE_ARG)[1];
	tseg1=(int)mxGetPr(CAN1_USER_BAUDRATE_ARG)[2];
	tseg2=(int)mxGetPr(CAN1_USER_BAUDRATE_ARG)[3];
	
	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_initialize_chip_mb1(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip_mb1 \n");
      		return(-1);
    	}
		break;
	case 2:
		if (CANPC_initialize_chip_mb2(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip_mb2 \n");
      		return(-1);
    	}
		break;
	case 3:
		if (CANPC_initialize_chip_mb3(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip_mb3 \n");
      		return(-1);
    	}
		break;
	}
	
	presc=(int)mxGetPr(CAN2_USER_BAUDRATE_ARG)[0];
	sjw=(int)mxGetPr(CAN2_USER_BAUDRATE_ARG)[1];
	tseg1=(int)mxGetPr(CAN2_USER_BAUDRATE_ARG)[2];
	tseg2=(int)mxGetPr(CAN2_USER_BAUDRATE_ARG)[3];

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_initialize_chip2_mb1(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip2_mb1 \n");
      		return(-1);
    	}
		break;
	case 2:
		if (CANPC_initialize_chip2_mb2(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip2_mb2 \n");
      		return(-1);
    	}
		break;
	case 3:
		if (CANPC_initialize_chip2_mb3(presc, sjw, tseg1, tseg2, 0) != 0) {
      		printf("-->Error in CANPC_initialize_chip2_mb3 \n");
      		return(-1);
    	}
		break;
	}

	// Set acceptance filter for CAN 1 and 2   
	
	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
	   	if (CANPC_set_acceptance_mb1(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
	   	if (CANPC_set_acceptance_mb2(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
	   	if (CANPC_set_acceptance_mb3(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance_mb3 \n");
	      	return(-1);
	    }
		break;
	}

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
	   	if (CANPC_set_acceptance2_mb1(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance2_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
	   	if (CANPC_set_acceptance2_mb2(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance2_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
	   	if (CANPC_set_acceptance2_mb3(ACCEPT_MASK_1, ACCEPT_CODE_1, ACCEPT_MASK_XTD_1, ACCEPT_CODE_XTD_1) != 0) {
	     	printf("-->Error in CANPC_set_acceptance2_mb3 \n");
	      	return(-1);
	    }
		break;
	}
	
   	// Set resynchronization

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_set_mode_mb1(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
		if (CANPC_set_mode_mb2(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
		if (CANPC_set_mode_mb3(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode_mb3 \n");
	      	return(-1);
	    }
		break;
	}

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_set_mode2_mb1(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode2_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
		if (CANPC_set_mode2_mb2(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode2_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
		if (CANPC_set_mode2_mb3(SLEEPMODE_1, SPEEDMODE_1) != 0) {
	      	printf("-->Error in CANPC_set_mode2_mb3 \n");
	      	return(-1);
	    }
		break;
	}

   	// Set output control (phys. layer -1: default  CAN High Speed)                               

	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
	   	if (CANPC_set_output_control_mb1(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
		if (CANPC_set_output_control_mb2(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
		if (CANPC_set_output_control_mb3(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control_mb3 \n");
	      	return(-1);
	    }
		break;
	}
	
	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
	   	if (CANPC_set_output_control2_mb1(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control2_mb1 \n");
	      	return(-1);
	    }
		break;
	case 2:
		if (CANPC_set_output_control2_mb2(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control2_mb2 \n");
	      	return(-1);
	    }
		break;
	case 3:
		if (CANPC_set_output_control2_mb3(OUTPUT_CONTROL_1) != 0) {
	      	printf("-->Error in CANPC_set_output_control2_mb3 \n");
	      	return(-1);
	    }
		break;
	}

	return(0);

}

#endif



static void mdlInitializeSizes(SimStruct *S)
{
	int_T i;

    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		sprintf(msg,"Wrong number of input arguments passed.\n%d arguments are expected\n",NUMBER_OF_ARGS);
        ssSetErrorStatus(S,msg);
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

	if(mxGetPr(BUS_OFF_REC_ARG)[0]==3)
	{
		if( !ssSetNumInputPorts(S, 1) ) return;
		ssSetInputPortWidth(S, 0, 1);
		ssSetInputPortDirectFeedThrough(S, 0, 1);
		ssSetInputPortRequiredContiguous(S, 0, 1);
	}else
	{
		if( !ssSetNumInputPorts(S, 0) ) return;
	}

	if(mxGetPr(BUS_OFF_OUT_ARG)[0]==1)
	{
		if( !ssSetNumOutputPorts(S, 1) ) return;
		ssSetOutputPortWidth(S, 0, 1);
	}
	else
	{
		if( !ssSetNumOutputPorts(S, 0) ) return;
	}


    ssSetNumSampleTimes(S, 1);

    ssSetSimStateCompliance( S, HAS_NO_SIM_STATE );

    ssSetNumRWork(S, NO_R_WORKS);
    ssSetNumIWork(S, NO_I_WORKS);
    ssSetNumPWork(S, 0);

    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

	for (i=0;i<NUMBER_OF_ARGS;i++) {
		ssSetSFcnParamNotTunable(S,i);
	}

    ssSetOptions(S, SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME | SS_OPTION_EXCEPTION_FREE_CODE);
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S, 0, 0.0);
}



#define MDL_START
static void mdlStart(SimStruct *S)
{

#ifndef MATLAB_MEX_FILE

    int_T 			i,j;
	CANPC_RESSOURCES crRessources;
	int 			ret;

	int				frc;

		

	crRessources.uIOAdress= (int_T)mxGetPr(IO_BASE_ADDRESS_ARG)[0];
	crRessources.uInterrupt= (int_T)mxGetPr(INTNO_ARG)[0];
	crRessources.uRegisterBase= 1;
	crRessources.ulDPRMemBase= (int_T)mxGetPr(MEM_BASE_ADDRESS_ARG)[0];

	if (!CANAC2_FIRMWARE_LOADED[(int_T)mxGetPr(BOARD_ARG)[0]-1]) {

		switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
		case 1:
			 if ((ret = INIPC_initialize_board_mb1(&crRessources)) != 0) {
	    		sprintf(msg,"CAN-AC-104 Board 1: I/O Base Address may be wrong");
	        	ssSetErrorStatus(S,msg);
				return;
			}
			break;
		case 2:
			 if ((ret = INIPC_initialize_board_mb2(&crRessources)) != 0) {
	    		sprintf(msg,"CAN-AC-104 Board 2: I/O Base Address may be wrong");
	        	ssSetErrorStatus(S,msg);
				return;
			}
			break;
		case 3:
			 if ((ret = INIPC_initialize_board_mb3(&crRessources)) != 0) {
	    		sprintf(msg,"CAN-AC-104 Board 3: I/O Base Address may be wrong");
	        	ssSetErrorStatus(S,msg);
				return;
			}
			break;
		}

		// Load firmware
		
		switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
		case 1:
			printf("        downloading CAN-AC2-104 firmware on board 1\n");
			if ((ret = CANPC_reset_board_mb1()) != 0) {
	   			printf("-->Error in CANPC_reset_board_mb1: ");
	   			printResetErrorText(ret);
	   			return;
			}
			printf("        downloading CAN-AC2-104 firmware finished\n");
			break;
		case 2:
			printf("        downloading CAN-AC2-104 firmware on board 2\n");
			if ((ret = CANPC_reset_board_mb2()) != 0) {
	   			printf("-->Error in CANPC_reset_board_mb2: ");
	   			printResetErrorText(ret);
	   			return;
			}
			printf("        downloading CAN-AC2-104 firmware finished\n");
			break;
		case 3:
			printf("        downloading CAN-AC2-104 firmware on board 3\n");
			if ((ret = CANPC_reset_board_mb3()) != 0) {
	   			printf("-->Error in CANPC_reset_board_mb3: ");
	   			printResetErrorText(ret);
	   			return;
			}
			printf("        downloading CAN-AC2-104 firmware finished\n");
			break;
		}

		CANAC2_FIRMWARE_LOADED[(int_T)mxGetPr(BOARD_ARG)[0]-1]=1;
	    
	}

	// Initialization of the CAN parameters for CAN channel 1 and 2
	
	if (Initialize_can_parameter(S)) {
   		printf("-->Initialize_can_parameter failed \n");
   		return;
	}




    // Enable dynamic object buffer mode

    if(Initialize_DOB_mode(S))
    {
        printf("-->Initialize_DOB_mode failed \n");
        return;        
    }
    
    // Start Chip

    if(Start_chip(S))
    {
        printf("-->Start_chip failed \n");
        return;
    }

	// Excute Initialization if necessary

	{
		int_T 	start;
		uchar_T data[8];

		if ((int_T)mxGetPr(CAN_INIT_ARG)[0]) {

			start=1;

			for (i=0;i<(int_T)mxGetPr(CAN_INIT_ARG)[0];i++) {

				for(j=0;j<(int_T)mxGetPr(CAN_INIT_ARG)[start+3];j++) {
					data[j]=(uchar_T)mxGetPr(CAN_INIT_ARG)[start+4+j];
				}
				
				if (((int_T)mxGetPr(CAN_INIT_ARG)[start])==1) {  //CAN port 1
			 	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
					case 1:
						frc = CANPC_write_object_mb1((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb1 CAN 1: %d\n",frc);
							return;
						} 
						break;
					case 2:
						frc = CANPC_write_object_mb2((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb2 CAN 1: %d\n",frc);
							return;
						} 
						break;
					case 3:
						frc = CANPC_write_object_mb3((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb3 CAN 1: %d\n",frc);
							return;
						} 
						break;
					}
				} else {
					switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
					case 1:
						frc = CANPC_write_object2_mb1((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb1 CAN 2: %d\n",frc);
							return;
						} 
						break;
					case 2:
						frc = CANPC_write_object2_mb2((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb2 CAN 2: %d\n",frc);
							return;
						} 
						break;
					case 3:
						frc = CANPC_write_object2_mb3((int_T)mxGetPr(CAN_INIT_ARG)[start+2], (int_T)mxGetPr(CAN_INIT_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb3 CAN 2: %d\n",frc);
							return;
						} 
						break;
					} 
				}

            	//wait specified ms
				rl32eWaitDouble(mxGetPr(CAN_INIT_ARG)[start+12]);

				start+=13;

			}

		}

	}

					
    if(mxGetPr(BUS_OFF_REC_ARG)[0]!=1)
    {	
		/*Create the semaphore that will be used to signal CAN recovery thread when a bus-off occurs*/
		sprintf(sem_name,"canac2_sem_%d",(int_T)mxGetPr(BOARD_ARG)[0]);
		bus_status_semaphore = CreateSemaphore( NULL, 0,1,sem_name);

		/*Create background thread that recovers can board if goes to bus-off state*/
		can_recovery_thread = CreateThread(NULL, 0, canRecoveryFunc, S, 0, 0);
	}

#endif

}

#ifndef MATLAB_MEX_FILE
static DWORD WINAPI canRecoveryFunc( LPVOID param ) {

	SimStruct *S = (SimStruct *)param;

	while(1)
	{

		WaitForSingleObject(bus_status_semaphore,INFINITE);

		if(g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1])
		{

			Sleep(250);

			CANAC2_FIRMWARE_LOADED[(int_T)mxGetPr(BOARD_ARG)[0]-1]=0;

			if (Initialize_can_parameter(S)) 
			{
				g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 0;			
				continue;
			}
			
			CANAC2_FIRMWARE_LOADED[(int_T)mxGetPr(BOARD_ARG)[0]-1]=1;

			// Enable dynamic object buffer mode

			if(Initialize_DOB_mode(S))
			{
				g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 0;			
				continue;      
			}

			// Start Chip

			if(Start_chip(S))
			{
				g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 0;			
				continue;
			}

			g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 0;
			
		}
	}
    return 0;
}
#endif
static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE

	static FILETIME start_time;
    static int32_T bus_off_detected = 0;

    if(mxGetPr(BUS_OFF_OUT_ARG)[0]==1 || mxGetPr(BUS_OFF_REC_ARG)[0]!=1)
    {	
		if(!g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1])
		{
			/*recovery thread: idle*/

			int32_T bus_state;
			/* Check for Bus Off*/
	        
			switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
			  case 1:
					bus_state = CANPC_get_bus_state_mb1(1) | CANPC_get_bus_state_mb1(2);
				break;
			  case 2:
					bus_state = CANPC_get_bus_state_mb2(1) | CANPC_get_bus_state_mb2(2); 
				break;
			  case 3:
					bus_state = CANPC_get_bus_state_mb3(1) | CANPC_get_bus_state_mb3(2); 
				break;
			}
	        

			if(bus_state>=2)
			{

				if(	mxGetPr(BUS_OFF_REC_ARG)[0]==1 )
				{
					bus_off_detected = 1;
				}
				else if(mxGetPr(BUS_OFF_REC_ARG)[0]==2 )
				{
					/*Auto Recovery*/
					FILETIME current_time;
					uint32_T elapsed_time_ms = 0;

					GetSystemTimeAsFileTime(&current_time);

					if(!bus_off_detected)
					{
						
						start_time = current_time;
						bus_off_detected = 1;
					}



					elapsed_time_ms = ((current_time.dwHighDateTime - start_time.dwHighDateTime)/10000)*(2^32) + ((current_time.dwLowDateTime - start_time.dwLowDateTime)/10000);

					if (CAN_RECOVERY_DELAY <= elapsed_time_ms) {
						/*Recover from Bus-Off State*/
						bus_off_detected = 0;
						g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 1;
						ReleaseSemaphore(bus_status_semaphore,1,NULL);
					}
				}else
				{
					real_T* u=(real_T*)ssGetInputPortSignal(S,0);

					bus_off_detected = 1;

					/*check input for recovery triggering*/
					if(u[0]==1)
					{
						bus_off_detected = 0;
						/*Recover from Bus-Off State*/
						g_xpccanac2busstatus[(int_T)mxGetPr(BOARD_ARG)[0]-1] = 1;
						ReleaseSemaphore(bus_status_semaphore,1,NULL);
					}
				}

			}else
			{

				
				bus_off_detected = 0;
			}

		}

	}

	if(mxGetPr(BUS_OFF_OUT_ARG)[0]==1)
	{
		real_T* y=(real_T*)ssGetOutputPortSignal(S,0);

		if(bus_off_detected)
		{
			y[0] = 1;
		}else
		{
			y[0] = 0;
		}
	}

#endif    
}

/* Function to compute model update */
 
static void mdlTerminate(SimStruct *S)
{
            
#ifndef MATLAB_MEX_FILE

	// Excute Termination if necessary

	{
		int_T 	i,j,frc,start;
		uchar_T data[8];

		if ((int_T)mxGetPr(CAN_TERM_ARG)[0]) {

			start=1;

			for (i=0;i<(int_T)mxGetPr(CAN_TERM_ARG)[0];i++) {

				for (j=0;j<(int_T)mxGetPr(CAN_TERM_ARG)[start+3];j++) {
					data[j]=(uchar_T)mxGetPr(CAN_TERM_ARG)[start+4+j];
				}

				if (((int_T)mxGetPr(CAN_TERM_ARG)[start])==1) {  //CAN port 1
			 	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
					case 1:
						frc = CANPC_write_object_mb1((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb1 CAN 1: %d\n",frc);
							return;
						} 
						break;
					case 2:
						frc = CANPC_write_object_mb2((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb2 CAN 1: %d\n",frc);
							return;
						} 
						break;
					case 3:
						frc = CANPC_write_object_mb3((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb3 CAN 1: %d\n",frc);
							return;
						} 
						break;
					}
				} else {
					switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
					case 1:
						frc = CANPC_write_object2_mb1((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb1 CAN 2: %d\n",frc);
							return;
						} 
						break;
					case 2:
						frc = CANPC_write_object2_mb2((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb2 CAN 2: %d\n",frc);
							return;
						} 
						break;
					case 3:
						frc = CANPC_write_object2_mb3((int_T)mxGetPr(CAN_TERM_ARG)[start+2], (int_T)mxGetPr(CAN_TERM_ARG)[start+3], data);
						if (frc<0) {
							printf("ERROR:  CANPC_write_object_mb3 CAN 2: %d\n",frc);
							return;
						} 
						break;
					} 
				}


            	//wait specified ms
				rl32eWaitDouble(mxGetPr(CAN_TERM_ARG)[start+12]);

				start+=13;

			}

		}

	}

 	// Reinitialize
	switch ((int_T)mxGetPr(BOARD_ARG)[0]) {
	case 1:
		if (CANPC_reinitialize_mb1() != 0) {
	    	printf("-->Error in CANPC_reinitialize_mb1 \n");
			return;
		}
		break;
	case 2:
		if (CANPC_reinitialize_mb2() != 0) {
	    	printf("-->Error in CANPC_reinitialize_mb2 \n");
			return;
		}
		break;
	case 3:
		if (CANPC_reinitialize_mb3() != 0) {
	    	printf("-->Error in CANPC_reinitialize_mb3 \n");
			return;
		}		
		break;
	}

    if(mxGetPr(BUS_OFF_REC_ARG)[0]!=1)
    {	
	    TerminateThread(can_recovery_thread,0);
        CloseHandle(can_recovery_thread);
        CloseHandle(bus_status_semaphore);    
	}


#endif
  
}

#ifdef	MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

