
/*
	Relevant firmware history:

  58113:
	boot firmware 0.2 bugs:
	
	  1. bootloader prevented from running if CAN mode is non-zero.
         default CanMode was changed to 0xC000 20kbit in app firmware v0.9 and boot firmware 0.2.

	  2. last word fails to flash to address validation bug
*/

#define PMD_W32SERIAL_INTERFACE
// define COM_ADDRESS only for multi-drop serial communication.
//#define COM_ADDRESS 1

#define PMD_CAN_INTERFACE
#define PMD_PCI_INTERFACE
#define NODE_ID 0
#define CAN_TIMEOUT 200


//#define PMD_PCI_INTERFACE
#define PCI_BOARD 2


// If this macro is defined then the program will attempt to read
// defaults, then write them after flashing firmware.  Newer firmware
// can store manufacturing defaults in the boot sector, making this option
// unnecessary.
//#define KEEP_DEFAULTS

#include "C-Motion.h"
#include "C-Motionspecial.h"
#include "PMDsys.h"
#include "PMDdiag.h"	// for PMDGetErrorMessage
#include "PMDocode.h" // for PMDOPReset
#include "PMDutil.h"	// for PMDCopyAxisInterface


// This procedure is defined in PMDpci.c
extern PMDresult PMDPCI_WriteCMD(void* transport_data, PMDuint16 command);

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>

#ifdef REALLY_VERBOSE
# define TRYCALL_SUCCESS(_res, _cmd) PMDprintf("  %s\n", #_cmd)
#else
# define TRYCALL_SUCCESS(_res, _cmd) /**/
#endif

#ifdef _DEBUG
#define TRYCALL(_res, _cmd) {_res = _cmd; \
		if (PMD_NOERROR != _res && PMD_ERR_Reset != _res) { \
			PMDprintf("\nFAIL: L%d, %s, %s (%#x)\n", __LINE__, #_cmd, PMDGetErrorMessage(_res), _res); /*return _res;*/ } \
		else {TRYCALL_SUCCESS(_res, _cmd);}}
#else
#define TRYCALL(_res, _cmd) {_res = _cmd; \
	if (PMD_NOERROR != _res && PMD_ERR_Reset != _res) {\
		PMDprintf("\nFAIL: L%d, %s (%#x)\n", __LINE__, PMDGetErrorMessage(_res), _res); return _res; }}
#endif

enum {
	PMDDriveFlashMode = 0,
	PMDDriveFlashEraseNVRAM = 1,
	PMDDriveFlashWrite = 2,
	PMDDriveFlashWriteBegin = 3,
	PMDDriveFlashWriteEnd = 4,
	PMDDriveFlashSkip = 8,
	PMDDriveFlashEraseFirmware = 9,		// key 0x31A2
	PMDDriveFlashEraseBoot = 10,		// key 0x84B2
	PMDDriveFlashUnlockNVRAM = 11,
	PMDDriveFlashUnlockFirmware = 12,
	PMDDriveFlashUnlockBoot = 13,
	PMDDriveFlashUnlockOTP = 14,
	PMDDriveFlashOldDecodeMode = 15,
	PMD58113FlashMode = 0x100
};

enum {
	FirmwareInfo_App	= 254,
	FirmwareInfo_Boot	= 253
};

union {
	PMDuint8 bytes[2];
	PMDuint16 word;
} flash_buffer[0x8000];

// Needed for various workarounds.
int old_boot_code = 1;
int flash_boot_code = 0;
int bulk_mode = 0;
int bulk_mode_set = 0;
int bAtlas = 0;
int bSerial = 0;
int bCAN = 0;
int bPCI = 0;
long SerialBaud = 115200;
int EraseNVRAM = 0;


size_t flash_length = 0;

PMDresult atlas_wait(PMDAxisHandle *hAxis, PMDAxisHandle *hAtlas, int timeout, PMDresult expected_error){return PMD_NOERROR;}
PMDresult atlas_flash(PMDAxisHandle *hAxis, PMDuint16 *data, int length);
PMDresult read_file(char *filename);
PMDresult verify_key(PMDAxisHandle *hAxis, PMDuint16 buf[]);
PMDresult PMDDriveFlash(PMDAxisHandle *hAxis, PMDuint16 op, PMDuint16 arg)
{
	return SendCommandWordWord(hAxis, 0x30, op, arg);
}

// Atlas records the first two errors, and returns them in one 16-bit word.
PMDresult PMDGetInstructionErrors(PMDAxisHandle *hAxis, PMDuint16 *err1, PMDuint16 *err2)
{
	PMDuint16 err;
	PMDresult res = SendCommandGetWord(hAxis, PMDOPGetInstructionError, &err);
	if (PMD_NOERROR == res) {
		if (old_boot_code) {
			*err1 = 0;
			*err2 = 0;
		}
		else {
			*err1 = err & 0xFF;
			*err2 = (err >> 8) & 0xFF;
		}
	}
	return res;
}

// Second call to ignite returns success for Atlas.
PMDresult PMDIgnite(PMDAxisHandle *hAxis)
{
	PMDresult res = SendCommandWord(hAxis, 0xFE, 0x4A47);
	PMDuint16 err_code;

 tail:
	switch (res) {
	default:
		return res;

	case PMD_ERR_InvalidInstruction:
		res = SendCommandWord(hAxis, 0xFE, 0x4A47);
		break;

	case PMD_ERR_CommandError:
	case PMD_ERR_ChecksumError:
		if (PMD_NOERROR == PMDGetInstructionError(hAxis, &err_code)) {
			res = err_code;
			goto tail;
		}
		return res;
	}
	// Read the bad opcode error.
	PMDGetInstructionError(hAxis, &err_code);
	return res;
}

BOOL IsBootLoader(PMDAxisHandle *phAxis)
{
	PMDresult result;
	BOOL bBoot = FALSE;
	PMDuint32 version;

	result = GetVersionDWord(phAxis, &version);
	if (version == 0)
	{
		result = PMDGetProductInfo(phAxis, 0, &version); // index 0 returns currently running firmware 0=motion, 1=bootloader
		bBoot = (version == 1);
	}
	else
		bBoot = ((version & 0x00040000) == 0x00040000);

	return bBoot;
}


PMDresult DisplayVersionEx(PMDAxisHandle* phAxis)
{
    PMDresult result = PMD_NOERROR;
	char cPartNumber[20];
	char* pPartNumber = &cPartNumber[0];
	int i,j;
//    PMDuint16 generation, motorType, numberAxes, special;
    PMDuint16 custom, major, minor;
	PMDuint32 version;
	const char sFirmware[2][20] = {"Application", "Boot"};
	int bootoffset = 0;

//    PMD_RESULT(PMDGetVersion(phAxis, &generation, &motorType, &numberAxes, &special, &custom, &major, &minor));
    PMD_RESULT(PMDGetVersion32(phAxis, &version));

    if (result == PMD_NOERROR)
    {
		if (version != 0)
		{
			PMDprintf("Version 0x%08X\n", version);
//			PMDprintf("PMD processor MC%d%d%d%d0 v%d.%d.%d\n\n", generation, motorType, numberAxes, special, custom, major, minor);
		}
		else
		{
			for (j=0; j<1; j++)
			{
				bootoffset = j * 0x100;
				for (i=0; i<4; i++)
				{
					PMD_RESULT(PMDGetProductInfo(phAxis,  PMDProductInfoPartNumberWord1+i, (PMDuint32*)pPartNumber+i));
				}
				cPartNumber[19] = 0;
				PMD_RESULT(PMDGetProductInfo(phAxis,  PMDProductInfoVersion + bootoffset, &version));
				minor = (version >> 0) & 0xFF;
				major = (version >> 8) & 0xFF;
				custom = (version >> 16) & 0xFF;

				PMDprintf("%s firmware part number %s v%d.%d.%d\n\n", sFirmware[j], cPartNumber, custom, major, minor);
			}
		}
	}
	
	return result;
}

/*
	PMDGetTraceValue allows us to provde an extensible version
	facility:

	Each 32-bit value from trace variable 254 consists of 2 16-bit
	fields, the high word is an ASCII character giving the meaning
	of the value, and the low word is the value.	The start of the
	version block is indicated by both words zero.	The end of
	the version block is indicated by both words 0xFFFF.

    The boot checksum returned by GetTraceValue 253 is the one stored 
	in flash. This value is computed at startup and flashed if the 
	location is all F's. v0.4 and above.
*/
PMDresult long_version(PMDAxisHandle *hAxis, PMDuint8 tracevalue)
{
	PMDresult result;
	PMDuint32 v;
	PMDuint16 val, c, clast = 0;
	int i;

	i = 0;
	do {
		TRYCALL(result, PMDGetTraceValue(hAxis, tracevalue, &v));
	} while (i++ < 40 && (0 != v));

	i = 0;
	do {
		TRYCALL(result, PMDGetTraceValue(hAxis, tracevalue, &v));
		if (0xFFFFFFFF == v)
			break;
		c = (PMDuint16)(v >> 16);
		val = (PMDuint16)(0xFFFF & v);
		{
			if (c != clast)
				PMDprintf("\n");
			switch (c) {
			case 'v':
				if (c != clast)
					PMDprintf("           version: %04X", val);
				else
					PMDprintf("%04X", val);
				break;
			case 'b':
				PMDprintf("              beta: %04X", val);
				break;
			case 'y':
				PMDprintf("              year: %04X", val);
				break;
			case 'd':
				PMDprintf("              date: %02X/%02X", val>>8, val&0xFF);
				break;
			case 't':
				PMDprintf("              time: %02X:%02X", val>>8, val&0xFF);
				break;
			case '+':
				if (c != clast)
				PMDprintf("          checksum: %04X", val);
				else
				PMDprintf("%04X", val);
				break;
			case 'R':
				PMDprintf("        trace size: 0x%04X", val);
				break;
			case 'F':
				PMDprintf("   User NVRAM size: %04X", val);
				break;
			case 'M':
				PMDprintf("    Mfg NVRAM size: %04X", val);
				break;
			case 'O':
				PMDprintf("    OTP NVRAM size: %04X", val);
				break;
			case '"':
				if (c != clast)
				PMDprintf("                  : %c%c", val >> 8, val & 0xFF);
				else
				PMDprintf("%c%c", val >> 8, val & 0xFF);
				break;
			case 'f': 											
				PMDprintf("      FPGA version: 0x%04X", val);
				break;
			case 'u': 											
				if (c != clast)
				PMDprintf("         Unique ID: 0x%04X", val);
				else
				PMDprintf(" %04X", val);
				break;
#ifdef _DEBUG
			case 'k': 											// available only if ignited.
				PMDprintf("               key: 0x%04X", val);
				break;
			case 'c': 											// available only if ignited.
				PMDprintf("              freq: %d", val);				// SYSCLK MHz
				break;
			case 'l': 											// available only if ignited.
				PMDprintf("            locked: %d", val);				// SYSCLK MHz
				break;
			case 's': 											// available only if ignited.
				if (c != clast)
				PMDprintf("        Silicon ID: %04X", val);			// Silicon part id and revision
				else
				PMDprintf("  Rev: %04X", val);			// Silicon part id and revision
				break;
#endif
			default:
				if (c != clast)
					PMDprintf("                 %c: 0x%04Xh %dd", c, val, val);
				else
					PMDprintf("\n                    0x%04Xh %dd", val, val);
				break;
			}
		}
		clast = c;
	} while (i++ < 40);
	PMDprintf("\n");
	return result;
}

// C-Motion wants to set the axis number to zero when doing
// a reset, does not work for Atlas.
PMDresult PMDAtlasReset(PMDAxisHandle *hAxis)
{
	return SendCommand(hAxis, PMDOPReset);
}

PMDresult read_defaults(PMDAxisHandle *hAxis,
											 PMDAxisHandle *hAtlas,
											 PMDuint32 defaults[7])
{
	PMDresult result;

	TRYCALL(result, PMDIgnite(hAtlas));
	// Motor type
	TRYCALL(result, PMDGetDefault(hAxis, 0, &defaults[0]));
	// Analog offsets
	TRYCALL(result, PMDGetDefault(hAxis, 1, &defaults[1]));
	TRYCALL(result, PMDGetDefault(hAxis, 2, &defaults[2]));
	TRYCALL(result, PMDGetDefault(hAxis, 3, &defaults[3]));
	TRYCALL(result, PMDGetDefault(hAxis, 4, &defaults[4]));
	TRYCALL(result, PMDGetDefault(hAxis, 6, &defaults[6]));

	PMDprintf("%#x\n", defaults[0]);
	PMDprintf("%#x\n", defaults[1]);
	PMDprintf("%#x\n", defaults[2]);
	PMDprintf("%#x\n", defaults[3]);
	PMDprintf("%#x\n", defaults[4]);
	PMDprintf("%#x\n", defaults[6]);

	return result;
}

PMDresult write_defaults(PMDAxisHandle *hAxis,
												PMDAxisHandle *hAtlas,
												PMDuint32 defaults[7])
{
	PMDresult result;

	TRYCALL(result, PMDIgnite(hAtlas));

	// Motor type
	TRYCALL(result, PMDSetDefault(hAxis, 0, defaults[0]));

	// Atlas sometimes reports a bad checksum or bad opcode error
	// after SetDefault even though the command worked properly.
	// This should be fixed in the Atlas application
	// code, but for now we'll just ignore it.
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	// Analog offsets
	TRYCALL(result, PMDSetDefault(hAxis, 1, defaults[1]));
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	TRYCALL(result, PMDSetDefault(hAxis, 2, defaults[2]));
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	TRYCALL(result, PMDSetDefault(hAxis, 3, defaults[3]));
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	TRYCALL(result, PMDSetDefault(hAxis, 4, defaults[4]));
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	TRYCALL(result, PMDSetDefault(hAxis, 6, defaults[6]));
	atlas_wait(hAxis, hAtlas, 1000, 0xFFFF);

	return result;
}


void usage(int argc, char *argv[])
{
	
	if (argc > 2)
	{
		if (0 == strncmp(argv[2], "CAN", 3))
		{
			PMDprintf("usage: %s <filename> CAN <NodeID> <baud>\n", argv[0]);
			PMDprintf(" <NodeID> should be between 0 to 127\n");
			PMDprintf(" <baud> should be between 0 to 7\n");
			PMDprintf("   where 0=1M, 1=800K, 2=500K, 3=250K, 4=125K, 5=50K, 6=20K, 7=10K\n");
		}
	}
	else
	{
		PMDprintf("usage: %s <filename> <interface> <axis>\n", argv[0]);
		PMDprintf(" <interface> should be PCI, CAN, COM1, COM2, ...\n");
		PMDprintf(" <axis> should be 1, 2, 3, or 4 to specify Atlas\n");
	}
	exit(EXIT_FAILURE);
}


int main(int argc, char* argv[])
{
	PMDAxis AxisNum = 0;
#ifdef CMD_PROMPT
	char *filename = 0;
#else
	char filename[30];
#endif
	char *axis_name = 0;
	int com_port = 0;
	PMDAxisHandle hAxis;
	char sPort[20];
	int i = 3;
	int Node;
	int CANbaud;

	PMDuint32 vmajor, vminor;
	PMDuint32 version;
	PMDuint16 status;
	PMDresult result;

	
#ifdef CMD_PROMPT
	if (argc < 3)
		usage(argc, argv);
		
	if (argc > 1)
	{
		strcpy(sPort, argv[2]);
		sPort[0] = toupper(sPort[0]);
		sPort[1] = toupper(sPort[1]);
		sPort[2] = toupper(sPort[2]);
	}
	if (0 == strncmp(sPort, "CAN", 3))
		bCAN = 1;
	else if (0 == strncmp(sPort, "PCI", 3))
		bPCI = 1;
	else if (1 == sscanf(sPort, "COM%d", &com_port))
		bSerial = 1;
	else
		usage(argc, argv);

	
    while (argc > i)
	{
		if (1 == sscanf(argv[i], "bulk=%d", &bulk_mode))
		{
			if (bulk_mode)
				PMDprintf("Bulk mode enabled\n");
			else
				PMDprintf("Bulk mode disabled\n");
			bulk_mode_set = 1;
		}
		if (0 == strcmp(argv[i], "EraseNVRAM"))
		{
			EraseNVRAM = 1;
		}
		else
			axis_name = argv[i];
		i++;
	}
#else
		//bCAN = 1;
		bSerial = 1;
#endif

	if (axis_name != NULL && 1 == strlen(axis_name) && !bCAN)
	{
		PMDprintf("Atlas axis #%s\n", axis_name);
		bAtlas = 1;
		switch (*axis_name) 
		{
	//	default: usage(argc, argv);
		case '1':
			AxisNum = PMDAxis1;
			break;
		case '2':
			AxisNum = PMDAxis2;
			break;
		case '3':
			AxisNum = PMDAxis3;
			break;
		case '4':
			AxisNum = PMDAxis4;
			break;
		}

	}
	else
		PMDprintf("MC58113 firmware updater V2.8 Release\n");

#ifdef CMD_PROMPT
	filename = argv[1];
#else
	strcpy(filename,"531130012.bin");
#endif
	if (strstr(filename, "_boot"))
		flash_boot_code = 1;
	if (strstr(filename, "_debug") && !strstr(filename, "sION")) // bootloader with no password
		flash_boot_code = 1;

	if (flash_boot_code)
		PMDprintf("Detected boot firmware file to be flashed.\n");

	if (PMD_NOERROR != read_file(filename))
		exit(EXIT_FAILURE);

	PMDGetCMotionVersion( &vmajor, &vminor );

	PMDprintf("\nC-Motion Version %d.%d \n", vmajor, vminor);

	// open port
	if (bSerial)
	{
		/*
			 By default the serial interface will be set to COM1, 57600 and
			 the point-to-point protocol
			 The third parameter represents the COM port number (1=COM1)
		*/
		com_port = 3;
		PMDprintf("trying COM%d\n", com_port);
		result = PMDSetupAxisInterface_Serial(&hAxis, AxisNum, (PMDuint8)com_port);
		if (PMD_NOERROR != result) 
		{
			PMDprintf("Serial port initialization failed: %s\n", PMDGetErrorMessage(result));
			return 1;
		}
#if defined(COM_ADDRESS)
		{
			PMDSerial_SetProtocol(hAxis.transport_data, PMDSerialProtocolMultiDropUsingIdleLineDetection);
			PMDSerial_SetMultiDropAddress(hAxis.transport_data, COM_ADDRESS);
		}
#endif
	}
	else if (bCAN)
	{
		/*
			 Chip default CAN baud is 20kbit/s in versions >=0.9
		*/
#ifdef CMD_PROMPT
    	if (argc != 5)
			usage(argc, argv); 
		
		Node = atoi(argv[3]);
		CANbaud = atoi(argv[4]);
#else
		Node = 1;
		CANbaud = 6;
#endif
		if ((Node < 0) | (Node > 127) | (CANbaud < 0) | (CANbaud > 7)) usage(argc, argv);
		
		PMDprintf("trying CAN ID %d\n", Node);
		result = PMDSetupAxisInterface_CAN(&hAxis, AxisNum, CANbaud, Node);
		if (PMD_NOERROR != result) 
		{
			PMDprintf("CAN port initialization failed: %s\n", PMDGetErrorMessage(result));
			return 1;
		}
	}
	else if (bPCI)
	{
		PMDprintf("trying PCI board %d\n", PCI_BOARD);
		result = PMDSetupAxisInterface_PCI(&hAxis, AxisNum, PCI_BOARD);
		if (PMD_NOERROR != result) 
		{
			PMDprintf("PCI initialization failed: %s\n", PMDGetErrorMessage(result));
			return 1;
		}
	}

	result = GetVersionDWord(&hAxis, &version);

	if (PMD_NOERROR == result) 
	{
		int ch;
		PMDprintf("Current Magellan version %08X\n", version);

		// check communications
		// read error status to clear, needed for PCI.
		result = PMDGetInstructionError(&hAxis, &status);
//		if (PMD_NOERROR != status)
//				PMDprintf("Found prior error code %#x (%s)\n", status, PMDGetErrorMessage(status));
		if (!IsBootLoader(&hAxis)) // long_version does not work from bootloader
		{
			PMDprintf("\nCurrent application firmware information:\n");
			long_version(&hAxis, FirmwareInfo_App);
		}

		PMDprintf("Hit 'y' to continue with flashing or any other key to abort.\n");
		ch = getch();
		if (ch == 'y')
		{
			// do any baud rate changes here
			if (bSerial)
			{
#if 0 //def _DEBUG
				// increase baud to speed up flash process, 
				// but this doesn't work for customers passing serial data through an MCU
				result = PMDSetSerialPortMode(&hAxis, PMDSerialBaud115200, PMDSerialParityNone, 0, 0, 0);
				result = PMDSerial_SetConfig(hAxis.transport_data, 115200, PMDSerialParityNone);
#endif
				PMDSerial_SetTimeout(hAxis.transport_data, 500);
			}
#ifdef OLD_BOOT_VERSION
			else if (bCAN)
			{
				result = GetVersionDWord(&hAxis, &version);
				if (PMD_ERR_Timeout == result) 
					PMDprintf("trying 1Mb baud.\n");
				// changing baud to 1Mb also works around bug in bootloader that prevents the bootloader from running when CanMode != 0
				if (PMD_NOERROR == result) 
				{
					PMDprintf("switching to 1Mb baud.\n");
					result = PMDSetCANMode(&hAxis, NODE_ID, PMDCANBaud1000000);
				}
				PMDCAN_SetBaud(hAxis.transport_data, PMDCANBaud1000000);
				TRYCALL(result, GetVersionDWord(&hAxis, &version));
			}
#endif
			result = atlas_flash(&hAxis, (PMDuint16*)flash_buffer, flash_length);
		}
	}

	if (PMD_NOERROR == result)
	{
		DisplayVersionEx(&hAxis);
		if (!IsBootLoader(&hAxis)) // long_version does not work from bootloader
		{
#ifdef _DEBUG
			PMDIgnite(&hAxis); // to display extra version info
#endif
			PMDprintf("\nApplication firmware information:\n");
			long_version(&hAxis, FirmwareInfo_App);
			PMDprintf("\nBoot firmware information:\n");
			long_version(&hAxis, FirmwareInfo_Boot);
		}
	}
	else
	{
		PMDuint16 err_code;

		PMDprintf("ERROR: %s (%#X)\n", PMDGetErrorMessage(result), result);
		switch (result) 
		{
		default:
			break;
		case PMD_ERR_CommandError:
		case PMD_ERR_ChecksumError:
			result = PMDGetInstructionError(&hAxis, &err_code);
			if (PMD_NOERROR != err_code)
				PMDprintf("  %s (%#X)\n", PMDGetErrorMessage(result), result);
			break;
		}
	}
	// close the interface handle
	PMDCloseAxisInterface(&hAxis);

	return 0;
}



PMDresult atlas_flash(PMDAxisHandle *hAxis, PMDuint16 *data, int length)
{
	PMDresult result;
	int i, j, m;
	int ch;
	PMDuint32 checksum, old_version, new_version, boot_version;
	PMDuint16 err_code1, err_code2;
	PMDAxisHandle hAxisID0,*hAxisRestore=NULL;
//	PMDuint32 defaults[7];

	TRYCALL(result, GetVersionDWord(hAxis, &old_version));
	PMDprintf("Current version %08X\n", old_version);

	TRYCALL(result, PMDNoOperation(hAxis));
	TRYCALL(result, PMDGetInstructionErrors(hAxis, &err_code1, &err_code2));
	if (PMD_NOERROR != err_code1) 
	{
		PMDprintf("clearing previous errors: %s (%#0X), %s (%#0X)\n",
							PMDGetErrorMessage(err_code1), err_code1,
							PMDGetErrorMessage(err_code2), err_code2);
	}


//	TRYCALL(result, verify_key(hAxis, data));

#ifdef KEEP_DEFAULTS
	if (0 == (old_version & 0x40000)) 
	{
		PMDprintf("Reading defaults\n");
		read_defaults(hAxis, defaults);
	}
#endif

	// workaround for bug in bootloader 0.2 that prevents bootloader from running if CAN mode was non-zero
//	if (bSerial)
//		TRYCALL(result, PMDSetCANMode(hAxis, 0, PMDCANBaud1000000));

	PMDprintf("entering flash mode...");
	if (bAtlas)
		result = PMDDriveFlash(hAxis, PMDDriveFlashMode, 0);
	else
	{
		result = PMDDriveFlash(hAxis, PMD58113FlashMode, 0);
		if (result == PMD_ERR_InvalidParameter) // <0.9 firmware uses PMDDriveFlashMode
			result = PMDDriveFlash(hAxis, PMDDriveFlashMode, 0);
	}
	if (result != PMD_NOERROR)
	{
		puts(PMDGetErrorMessage(result));
		return result;
	}
	Sleep(300);

#ifdef OLD_BOOT_VERSION
	if (bCAN)  // for older boot firmware which did not save CanMode after a DriveFlash command?
	{
		result = GetVersionDWord(hAxis, &boot_version);
		if (PMD_ERR_Timeout == result) 
		{
			PMDCAN_SetBaud(hAxis->transport_data, PMDCANBaud1000000);
			result = GetVersionDWord(hAxis, &boot_version);
			if (PMD_NOERROR == result) 
			{
				result = PMDSetCANMode(hAxis, NODE_ID, PMDCANBaud1000000);
				PMDCAN_SetBaud(hAxis->transport_data, PMDCANBaud1000000);
			}
		}
	}
#endif
	result=GetVersionDWord(hAxis, &boot_version);
	
//	result = 1;   /// just for testing
//  For an unknown reason, the FlashMode command on the Raad Symtems causes a TI clock error and the active NodeID
//  will now be zero.  If the system CAN baud rate is not 20K this will not work anyways. 
	if (result)
	{
		PMDprintf("Trying CAN Node ID 0...\n");
		PMDCreateMultiDropHandle_CAN(&hAxisID0, hAxis, PMDAxis1, 0);
		hAxisRestore = hAxis;
		hAxis = &hAxisID0;
		TRYCALL(result, GetVersionDWord(hAxis, &boot_version));
		if (result) return result;
		if ((boot_version & 0x00040000) == 0)
		{
			PMDprintf("GetVersion at Node ID 0 indicates boot loader not running.\n");
			PMDprintf("entering flash mode Node ID 0...");


			result = PMDDriveFlash(hAxis, PMD58113FlashMode, 0);
			Sleep(200);
			TRYCALL(result, GetVersionDWord(hAxis, &boot_version));
		}

		PMDReset(hAxis);
		Sleep(300);   // The Raad board needs extra delay for some reason.
		
	    // Should now be back at original CAN settings
		hAxis = hAxisRestore;
		
	}

	boot_version = 0;
	TRYCALL(result, GetVersionDWord(hAxis, &boot_version));
	PMDprintf("Boot version querry at original CAN config returned %08X\n", boot_version);
 
	if ((boot_version & 0x00040000) == 0)
	{
		PMDprintf("GetVersion indicates boot loader not running.\n");
		return PMD_ERR_BadState;
	}
	// bulk mode was implemented in boot version 3 and up
	if (!bulk_mode_set)
	{
		if ( (boot_version & 0x000000FF) > 2)
			bulk_mode = 1;
		if (bCAN)
			bulk_mode = 0;
		if (bAtlas)
			bulk_mode = 1;
		if (flash_boot_code)
			bulk_mode = 0; // for easier error recovery
	}

	
erase:
	if (!bAtlas)
	{
		if (bSerial)
			PMDSerial_SetTimeout(hAxis->transport_data, 17000);
		if (bCAN)
			PMDCAN_SetTimeout(hAxis->transport_data, 17000);
	}
	if (flash_boot_code)
	{
		PMDprintf("erasing boot firmware\n");
		result = PMDDriveFlash(hAxis, PMDDriveFlashEraseBoot, 0x84B2);
		if (result != PMD_ERR_OK)
		{
			PMDprintf("\nError erasing flash: %s, (%#x)\n", PMDGetErrorMessage(result), result);  
			PMDprintf("'e' to erase and start over, 'a' to abort, any other key to continue\n");
			ch = getch();
			if (ch == 'e')
				goto erase;
			if (ch == 'q')
				return result;
		}
	}
	else
	{
		// erase NVRAM as well in case any commands become incompatible.
		if (EraseNVRAM)
		{
			PMDprintf("erasing NVRAM\n");
			TRYCALL(result, PMDDriveFlash(hAxis, PMDDriveFlashEraseNVRAM, 0));
		}
//		PMDprintf("unlocking application firmware\n");
//		TRYCALL(result, PMDDriveFlash(hAxis, PMDDriveFlashUnlockFirmware, 0x31A2));
		PMDprintf("erasing application firmware\n");
		TRYCALL(result, PMDDriveFlash(hAxis, PMDDriveFlashEraseFirmware, 0x31A2));
	}
	if (!bAtlas)
	{
		if (bSerial)
			PMDSerial_SetTimeout(hAxis->transport_data, 500);
		if (bCAN)
			PMDCAN_SetTimeout(hAxis->transport_data, 500);
	}

	PMDprintf("writing firmware\n");

	bulk_mode = 0;
	if (bulk_mode)
	{
		m = 1;
		for (i = 0; i < length; i += m) 
		{
			PMDprintf("%04X\r", i);
			fflush(stdout);
			if (i < length - 32)
				m = 32;
			else
				m = length - i;

			checksum = 0;
			for (j = 0; j < m; j++)
				checksum += data[i + j];
			while (0xFFFF0000 & checksum)
				checksum = (0xFFFF & checksum) + (checksum >> 16);
			TRYCALL(result, PMDDriveFlash(hAxis, PMDDriveFlashWriteBegin, (PMDuint16)m));
			for (j = 0; j < m; j++) 
			{
				PMDuint8 buf[3];
				DWORD nb;
				BOOL bres;

				buf[1] = 0xFF & (data[i + j] >> 8);
				buf[2] = 0xFF & data[i + j];

				if (bSerial)
				{
					PMDSerialIOData *transport_data = (PMDSerialIOData *)hAxis->transport_data;
#ifndef COM_ADDRESS
					bres = (WriteFile(transport_data->hPort, &buf[1], 2, &nb, NULL) && 2==nb);
#else
					buf[0] = COM_ADDRESS;
					bres = (WriteFile(transport_data->hPort, &buf[0], 3, &nb, NULL) && 3==nb);
#endif
					if (!bres) {
						result = PMD_ERR_CommPortWrite;
						PMDprintf("FAIL:  WriteFile\n");
						return result;
					}
				}
				else if (bCAN)
				{
					TRYCALL(result, TransmitData(hAxis->transport_data, buf, 2, CAN_TIMEOUT));
				}
				else if (bPCI)
				{
					TRYCALL(result, PMDPCI_WriteCMD(hAxis->transport_data, data[i + j]));
				}
			}

#if defined(COM_ADDRESS)
			if (bSerial)
			{
				// This resynchronization is required to get multi-drop serial to work.
				// I wish I knew why.
				j = 0;
				while (PMD_NOERROR != PMDNoOperation(hAxis) &&
							 j++ < 3) 
				{
					Sleep(1);
				}
			}
#endif

			TRYCALL(result, PMDDriveFlash(hAxis, PMDDriveFlashWriteEnd, (PMDuint16)checksum));
		}
	}
	else
	{
		for (i = 0; i < length; i+=1000) 
		{
			putchar('|');
		}
		putchar('\n');
		for (i = 0; i < length; i++) 
		{
			result = PMDDriveFlash(hAxis, PMDDriveFlashWrite, data[i]);
			if ((i % 1000) == 0)
				putchar('*');
			if (result != PMD_ERR_OK) // && i < 0x1fff)
			{
				PMDprintf("\nError writing byte #%d to flash: %s, (%#x)\n", i, PMDGetErrorMessage(result), result);  
				PMDprintf("'e' to erase and start over, 'q' to quit, any other key to continue\n");
				ch = getch();
				if (ch == 'e')
					goto erase;
				if (ch == 'q')
					return result;
			}

		}
	}
	PMDprintf("\nDone.\n\n");

	// TODO: do not reset when flashing boot loader if an error occured
	Sleep(1);
	PMDprintf("resetting... \n");
	TRYCALL(result, PMDAtlasReset(hAxis));

	Sleep(700);

	if (bSerial)
		result = PMDSerial_SetConfig(hAxis->transport_data, 57600, PMDSerialParityNone);

	if (bCAN)
	{
		result = GetVersionDWord(hAxis, &new_version);
		if (PMD_ERR_Timeout == result) 
		{
			
			// Depending on the how the default Node ID is set, (via SPI for example), it may neccessary to 
			// change the current Node ID  from 0 and restore it to whatever it as when this exectuable started.

			PMDprintf("Trying CAN Node ID 0...\n");
			
			PMDCANIOTransportData* pCANIOTransportData;
			int NodeID;
			pCANIOTransportData = (PMDCANIOTransportData*)hAxis->transport_data;
			NodeID = pCANIOTransportData->txID;
			NodeID -= 0x600;
			
			if(&hAxisID0!=NULL) hAxis = &hAxisID0;
			else
			{
				PMDprintf("Hanlde not initialized");
				return -1;
			}
			TRYCALL(result, GetVersionDWord(hAxis, &new_version));
			if (result) return result;
	
			PMDprintf("Restoring to NodeID %d...", NodeID);
			PMDSetCANMode(hAxis, NodeID, 6);
			Sleep(100);
			hAxis = hAxisRestore;
			result = GetVersionDWord(hAxis, &new_version);
			if (result) return result;
			PMDCloseAxisInterface(&hAxisID0);
		}
	}
	
	else TRYCALL(result, GetVersionDWord(hAxis, &new_version));

	PMDprintf("New version %08X\n", new_version);
/*
	if ((new_version & 0x00040000) == 0x00000000)
		long_version(hAxis); // does not work from bootloader 
	else if (!flash_boot_code)
	{
		TRYCALL(result, PMDAtlasReset(hAxis));
		Sleep(500);
	}
*/
#ifdef KEEP_DEFAULTS
	if (0 == (old_version & 0x40000)) {
		PMDprintf("Restoring defaults\n");
		write_defaults(hAxis, &hAtlas, defaults);
	}
#endif

	return result;
}

PMDresult read_file(char *filename)
{
	FILE *inp = fopen(filename, "rb");
	size_t i, n;

	if (!inp) {
		PMDprintf("ERROR opening file \"%s\": %s (%d)\n",
							filename, strerror(errno), errno);
		return PMD_ERR_InvalidOperation;
	}
	n = sizeof(flash_buffer)/sizeof(PMDuint16);
	n = fread(flash_buffer, sizeof(PMDuint16), n, inp);

	PMDprintf("%d words read from file \"%s\"\n", n, filename);

	/* Make life simple, don't assume endianness. */
	for (i = 0; i < n; i++)
		flash_buffer[i].word = flash_buffer[i].bytes[0] + (flash_buffer[i].bytes[1] << 8);
	flash_length = n;
	fclose(inp);
	return PMD_NOERROR;
}

#define VERSION_OFFSET 2				/* 32 bit version */
#define KEY_OFFSET 4						/* offset to 32 bit, little endian flash key. */
#define YEAR_OFFSET 6
#define DATE_OFFSET 7
#define TIME_OFFSET 8
#define CHECKSUM_OFFSET 10		 /* defined in 16 bit words, nskip must be just before checksum. */

// According to Wikipedia this is the glibc rand() implementation.
// Sounds ok to me, we're not really looking for strong crypto here.
// We'll use the top 16 bits.
#define KEY_MULTIPLIER 110351524
#define KEY_INCREMENT 12345
#define KEY_MASK 0x7FFFFFFF
#define KEY_SHIFT 15
#define SION_FLASHKEY 0x2F36CE86

PMDresult verify_key(PMDAxisHandle *hAxis, PMDuint16 *buf)
{
	PMDresult result;
	PMDuint32 key, key_state, key_read, key_mask;
	PMDuint32 v;
	PMDuint16 decode_buf[CHECKSUM_OFFSET + 2];
	int i;

	key = 0;
	TRYCALL(result, PMDIgnite(hAxis));
	TRYCALL(result, PMDIgnite(hAxis));
	for (i = 0; i < 20; i++) {
		TRYCALL(result, PMDGetTraceValue(hAxis, 254, &v));
		if ('k' == (v >> 16)) {
			key = (0xFFFF & v) << 16;
			TRYCALL(result, PMDGetTraceValue(hAxis, 254, &v));
			key |= (0xFFFF & v);
			break;
		}
	}
	if (key == 0)
	{
		PMDprintf("Key not found.\n");
		return PMD_ERR_InvalidParameter; // for lack of a better error
	}

	// PMDprintf("Firmware header\n");
	key_state = key;
	for (i = 0; i < CHECKSUM_OFFSET + 2; i++) {
		key_mask = (key_state >> KEY_SHIFT) & 0xFFFF;
		if (key)
			key_state = KEY_MASK & key_state * KEY_MULTIPLIER + KEY_INCREMENT;
		decode_buf[i] = (PMDuint16)(buf[i] ^ key_mask);
		//		PMDprintf("  %2d: %04X\n", i, decode_buf[i]);
	}

	key_read = ((PMDuint32)decode_buf[KEY_OFFSET] | (PMDuint32)decode_buf[KEY_OFFSET + 1] << 16);
	// PMDprintf("read key %#010X\n", key_read);
	if (key_read != key) {
		PMDprintf("Bad firmware encoding key or format\n");
		return 0x7000;
	}
	return result;
}

