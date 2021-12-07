/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define MAX_PAYLOAD_SIZE 256 //We need ~220 bytes for getProtocolVersion on most ublox modules

// const headers for all UBX messages
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;


////////////////////////////////////////////////////////
/////////////////// ubx Class Ids //////////////////////
////////////////////////////////////////////////////////
const uint8_t UBX_CLASS_NAV = 0x01;	 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_RXM = 0x02;	 //Receiver Manager Messages: Satellite Status, RTC Status
const uint8_t UBX_CLASS_INF = 0x04;	 //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ESF = 0x10;	 //External Sensor Fusion Messages
const uint8_t UBX_CLASS_ACK = 0x05;	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_UPD = 0x09;	 //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const uint8_t UBX_CLASS_CFG = 0x06;	 //Configuration Input Messages: Configure the receiver.
const uint8_t UBX_CLASS_MON = 0x0A;	 //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_TIM = 0x0D;	 //Timing Messages: Time Pulse Output, Time Mark Results
const uint8_t UBX_CLASS_MGA = 0x13;	 //Multiple GNSS Assistance Messages: Assistance data for various GNSS
const uint8_t UBX_CLASS_LOG = 0x21;	 //Logging Messages: Log creation, deletion, info and retrieval
const uint8_t UBX_CLASS_SEC = 0x27;	 //Security Feature Messages
const uint8_t UBX_CLASS_NMEA = 0xF0; //NMEA Strings: standard NMEA strings


////////////////////////////////////////////////////////
////////////////// ubx Message Ids /////////////////////
////////////////////////////////////////////////////////
const uint8_t UBX_ACK_ACK = 0x01; //Message acknowledged (Output)
const uint8_t UBX_ACK_NACK = 0x00; //Message not acknowledged (Output)

const uint8_t UBX_CFG_SPT;		//Configure and start a sensor production test (Get/Set)
const uint8_t UBX_CFG_RST = 0x04;		//Not included. Relic code that unit will not respond to.
const uint8_t UBX_CFG_VALDEL = 0x8C;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const uint8_t UBX_CFG_VALGET = 0x8B;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A;	//Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

const uint8_t UBX_ESF_ALG = 0x14; //IMU alignment information (Periodic/polled)
const uint8_t UBX_ESF_INS = 0x15; //Vehicle dynamics information (Periodic/polled)
const uint8_t UBX_ESF_MEAS = 0x02; //External sensor fusion measurements (Input/output)
const uint8_t UBX_ESF_RAW = 0x03; //Raw sensor measurements (Output)
const uint8_t UBX_ESF_STATUS = 0x10; //External sensor fusion status (Periodic/polled)

const uint8_t UBX_INF_DEBUG = 0x04;	  //ASCII output with debug contents
const uint8_t UBX_INF_ERROR = 0x00;	  //ASCII output with error contents
const uint8_t UBX_INF_NOTICE = 0x02;  //ASCII output with informational contents
const uint8_t UBX_INF_TEST = 0x03;	  //ASCII output with test contents
const uint8_t UBX_INF_WARNING = 0x01; //ASCII output with warning contents

const uint8_t UBX_MON_COMMS = 0x36; //Comm port information
const uint8_t UBX_MON_GNSS = 0x28;	//Information message major GNSS selection
const uint8_t UBX_MON_HW2 = 0x0B;	//Extended Hardware Status
const uint8_t UBX_MON_HW3 = 0x37;	//HW I/O pin information
const uint8_t UBX_MON_HW = 0x09;	//Hardware Status
const uint8_t UBX_MON_IO = 0x02;	//I/O Subsystem Status
const uint8_t UBX_MON_MSGPP = 0x06; //Message Parse and Process Status
const uint8_t UBX_MON_PATCH = 0x27; //Output information about installed patches
const uint8_t UBX_MON_RF = 0x38;	//RF information
const uint8_t UBX_MON_RXBUF = 0x07; //Receiver Buffer Status
const uint8_t UBX_MON_RXR = 0x21;	//Receiver Status Information
const uint8_t UBX_MON_SPAN = 0x31;	//Signal characteristics (Periodic/polled)
const uint8_t UBX_MON_SPT = 0x2f;	//Sensor production test (Polled)
const uint8_t UBX_MON_TXBUF = 0x08; //Transmitter Buffer Status. Used for query tx buffer size/state.
const uint8_t UBX_MON_VER = 0x04;	//Receiver/Software Version. Used for obtaining Protocol Version.

const uint8_t UBX_MGA_ACK_DATA0 = 0x60;		 //Multiple GNSS Acknowledge message
const uint8_t UBX_MGA_BDS_EPH = 0x03;		 //BDS Ephemeris Assistance
const uint8_t UBX_MGA_BDS_ALM = 0x03;		 //BDS Almanac Assistance
const uint8_t UBX_MGA_BDS_HEALTH = 0x03;	 //BDS Health Assistance
const uint8_t UBX_MGA_BDS_UTC = 0x03;		 //BDS UTC Assistance
const uint8_t UBX_MGA_BDS_IONO = 0x03;		 //BDS Ionospheric Assistance
const uint8_t UBX_MGA_DBD = 0x80;			 //Either: Poll the Navigation Database, or Navigation Database Dump Entry
const uint8_t UBX_MGA_GAL_EPH = 0x02;		 //Galileo Ephemeris Assistance
const uint8_t UBX_MGA_GAL_ALM = 0x02;		 //Galileo Almanac Assitance
const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02;	 //Galileo GPS time offset assistance
const uint8_t UBX_MGA_GAL_UTC = 0x02;		 //Galileo UTC Assistance
const uint8_t UBX_MGA_GLO_EPH = 0x06;		 //GLONASS Ephemeris Assistance
const uint8_t UBX_MGA_GLO_ALM = 0x06;		 //GLONASS Almanac Assistance
const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; //GLONASS Auxiliary Time Offset Assistance
const uint8_t UBX_MGA_GPS_EPH = 0x00;		 //GPS Ephemeris Assistance
const uint8_t UBX_MGA_GPS_ALM = 0x00;		 //GPS Almanac Assistance
const uint8_t UBX_MGA_GPS_HEALTH = 0x00;	 //GPS Health Assistance
const uint8_t UBX_MGA_GPS_UTC = 0x00;		 //GPS UTC Assistance
const uint8_t UBX_MGA_GPS_IONO = 0x00;		 //GPS Ionosphere Assistance
const uint8_t UBX_MGA_INI_POS_XYZ = 0x40;	 //Initial Position Assistance
const uint8_t UBX_MGA_INI_POS_LLH = 0x40;	 //Initial Position Assitance
const uint8_t UBX_MGA_INI_TIME_UTC = 0x40;	 //Initial Time Assistance
const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40;	 //Initial Time Assistance
const uint8_t UBX_MGA_INI_CLKD = 0x40;		 //Initial Clock Drift Assitance
const uint8_t UBX_MGA_INI_FREQ = 0x40;		 //Initial Frequency Assistance
const uint8_t UBX_MGA_INI_EOP = 0x40;		 //Earth Orientation Parameters Assistance
const uint8_t UBX_MGA_QZSS_EPH = 0x05;		 //QZSS Ephemeris Assistance
const uint8_t UBX_MGA_QZSS_ALM = 0x05;		 //QZSS Almanac Assistance
const uint8_t UBX_MGA_QZAA_HEALTH = 0x05;	 //QZSS Health Assistance

const uint8_t UBX_NAV_ATT = 0x05;		//Vehicle "Attitude" Solution
const uint8_t UBX_NAV_CLOCK = 0x22;		//Clock Solution
const uint8_t UBX_NAV_COV = 0x36;		//Covariance matrices
const uint8_t UBX_NAV_DOP = 0x04;		//Dilution of precision
const uint8_t UBX_NAV_EELL = 0x3d;		//Position error ellipse parameters (Periodic/polled)
const uint8_t UBX_NAV_EOE = 0x61;		//End of Epoch
const uint8_t UBX_NAV_GEOFENCE = 0x39;	//Geofencing status. Used to poll the geofence status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; //High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const uint8_t UBX_NAV_HPPOSLLH = 0x14;	//High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const uint8_t UBX_NAV_ORB = 0x34;		//GNSS Orbit Database Info
const uint8_t UBX_NAV_POSECEF = 0x01;	//Position Solution in ECEF
const uint8_t UBX_NAV_POSLLH = 0x02;	//Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07;		//All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const uint8_t UBX_NAV_RELPOSNED = 0x3C; //Relative Positioning Information in NED frame
const uint8_t UBX_NAV_RESETODO = 0x10;	//Reset odometer
const uint8_t UBX_NAV_SAT = 0x35;		//Satellite Information
const uint8_t UBX_NAV_SBAS = 0x32;		//SBAS status data (Periodic/polled)
const uint8_t UBX_NAV_STATUS = 0x03;	//Receiver Navigation Status
const uint8_t UBX_NAV_TIMEBDS = 0x24;	//BDS Time Solution
const uint8_t UBX_NAV_TIMEGAL = 0x25;	//Galileo Time Solution
const uint8_t UBX_NAV_TIMEGLO = 0x23;	//GLO Time Solution
const uint8_t UBX_NAV_TIMEGPS = 0x20;	//GPS Time Solution
const uint8_t UBX_NAV_TIMELS = 0x26;	//Leap second event information
const uint8_t UBX_NAV_TIMEQZSS = 0x27;  //QZSS time solution (Periodic/polled)
const uint8_t UBX_NAV_TIMEUTC = 0x21;	//UTC Time Solution
const uint8_t UBX_NAV_VELECEF = 0x11;	//Velocity Solution in ECEF
const uint8_t UBX_NAV_VELNED = 0x12;	//Velocity Solution in NED

const uint8_t UBX_RXM_MEASX = 0x14; //Satellite Measurements for RRLP
const uint8_t UBX_RXM_PMREQ = 0x41; //Requests a Power Management task (two differenent packet sizes)
const uint8_t UBX_RXM_RAWX = 0x15;	//Multi-GNSS Raw Measurement Data
const uint8_t UBX_RXM_RLM = 0x59;	//Galileo SAR Short-RLM report (two different packet sizes)
const uint8_t UBX_RXM_RTCM = 0x32;	//RTCM input status
const uint8_t UBX_RXM_SFRBX = 0x13; //Boradcast Navigation Data Subframe

const uint8_t UBX_SEC_UNIQID = 0x03; //Unique chip ID

const uint8_t UBX_TIM_TM2 = 0x03;  //Time mark data
const uint8_t UBX_TIM_TP = 0x01;   //Time Pulse Timedata
const uint8_t UBX_TIM_VRFY = 0x06; //Sourced Time Verification

const uint8_t UBX_UPD_SOS = 0x14; //Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup


////////////////////////////////////////////////////////
//////////////// ubx Message Lengths ///////////////////
////////////////////////////////////////////////////////
const uint16_t UBX_ACK_ACK_LENGTH = 2;
const uint16_t UBX_ACK_NACK_LENGTH = 2;

const uint16_t UBX_CFG_SPT_LENGTH = 12;
//Length currently unknown for UBX_CFG_VALDEL
//Length currently unknown for UBX_CFG_VALGET
//Length currently unknown for UBX_CFG_VALSET

const uint16_t UBX_ESF_ALG_LEN = 16;


////////////////////////////////////////////////////////
////////////////// ubxPacket Enums /////////////////////
////////////////////////////////////////////////////////
//Additional flags and pointers that need to be stored with each message type
struct ubxAutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1; // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1; // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1; // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh?
    } bits;
  } flags;
};

// ubxPacket validity
typedef enum{
	SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
	SFE_UBLOX_PACKET_VALIDITY_VALID,
	SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
}sfe_ublox_packet_validity_e;

// ubxPacket structure
typedef struct{
	uint8_t clsId; //Class ID of the UBX message
	uint8_t msgId; //Message ID of the UBX message
	uint16_t len; //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter; //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	sfe_ublox_packet_validity_e valid; //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
}ubxPacket;

////////////////////////////////////////////////////////
////////////////// ubxAck Messages /////////////////////
////////////////////////////////////////////////////////

/*
	Message : UBX_ACK_ACK - Message acknowledged
  	Type : Output
	Comment : Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at least within one second.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||    Payload    || Checksum
 			  0xb5 0x62 ||  0x05 || 0x01 ||       2        ||    UBX_ACK    || CK_A CK_B
 */

/*
	Message : UBX_ACK_NACK - Message not acknowledged
  	Type : Output
	Comment : Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at least within one second.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			  0xb5 0x62 ||  0x05 || 0x00 ||       2        ||     UBX_ACK    || CK_A CK_B
 */

////////////////////////////////////////////////////////
////////////////// ubxCfg Messages /////////////////////
////////////////////////////////////////////////////////

/*
	Message : UBX_CFG_SPT - Configure and start a sensor production test
  	Type : Get/set
	Comment : The production test uses the built-in self-test capabilities of an attached sensor.
			  This message is only supported if a sensor is directly connected to the u-blox receiver.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			  0xb5 0x62 ||  0x06 || 0x64 ||      12        ||   UBX_CFG_SPT  || CK_A CK_B
 */

/*
	Message : UBX_CFG_VALDEL - Delete configuration item values
  	Type : Set
	Comment : See 3.10.3.1 in ublox_interface_description on page 53
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			  0xb5 0x62 ||  0x06 || 0x8c ||  4 + [0..n]*4  || UBX_CFG_VALDEL || CK_A CK_B
 */

/*
	Message : UBX_CFG_VALGET - Message not acknowledged
  	Type : Output
	Comment : Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at least within one second.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			  0xb5 0x62 ||  0x06 || 0x8b ||  4 + [0..n]    || UBX_CFG_VALGET || CK_A CK_B
 */

/*
	Message : UBX_CFG_VALSET - Message not acknowledged
  	Type : Output
	Comment : Output upon processing of an input message. A UBX-ACK-ACK is sent as soon as possible but at least within one second.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			  0xb5 0x62 ||  0x06 || 0x8a ||   4 + [0..n]   || UBX_CFG_VALSET || CK_A CK_B
 */

////////////////////////////////////////////////////////
////////////////// ubxEsf Message///////////////////////
////////////////////////////////////////////////////////


/*
	Message : UBX_ESF_ALG - IMU alignment information
  	Type : Periodic/polled
  	Comment : This message outputs the IMU alignment angles which define the rotation from the installation-frame to the
			  IMU-frame. In addition, it indicates the automatic IMU-mount alignment status.
	Structure : Header  || Class ||  ID  || Length (Bytes) ||       Payload      || Checksum
 			  0xb5 0x62 ||  0x10 || 0x14 ||		  16	   || UBX_ESF_ALG_data_t || CK_A CK_B
 */
typedef struct{
	uint32_t iTow; //[ms] GPS time of week of the navigation epoch.
	uint8_t version; //Message version (0x01 for this version)
	union{
		struct{
			uint8_t autoMntAlgOn : 1; // Automatic IMU-mount alignment on/off bit
			uint8_t status : 3; // Status of the IMU-mount alignment
			                    //   0: user-defined/fixed angles are used
								//   1: IMU-mount roll/pitch angles alignment is ongoing
							    //   2: IMU-mount roll/pitch/yaw angles alignment is ongoing
								//   3: coarse IMU-mount alignment are used
								//   4: fine IMU-mount alignment are used
		}bits; //Bits in these flags
	}flags; //Flags
}UBX_ESF_ALG_data_t;



/*
	Message : UBX_ESF_INS
  	Type :
	Comment : .
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			            ||   	 ||  	 ||		 		   ||				 ||
 */

/*
	Message : UBX_ESF_MEAS
  	Type :
	Comment : .
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			            ||   	 ||  	 ||		 		   ||				 ||
 */

/*
	Message : UBX_ESF_RAW
  	Type :
	Comment : .
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			            ||   	 ||  	 ||		 		   ||				 ||
 */

/*
	Message : UBX_ESF_STATUS
  	Type :
	Comment : .
	Structure : Header  || Class ||  ID  || Length (Bytes) ||     Payload    || Checksum
 			            ||   	 ||  	 ||		 		   ||				 ||
 */
////////////////////////////////////////////////////////
////////////////// ubxInf Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxMga Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxMon Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxNav Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxRxm Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxSec Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxTim Message///////////////////////
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
////////////////// ubxUpd Message///////////////////////
////////////////////////////////////////////////////////




/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c2;



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);



int main(void){

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C2_Init();

  // Verify connection with ublox module


  // Create Incoming Messages from ulbox module



  while (1){




  };
};


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}


static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 38400;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0x20510001;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}


static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

