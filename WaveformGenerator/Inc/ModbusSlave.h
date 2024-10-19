/**************************************************************************************************************
 *                  File Name       :ModbusSlave.h
 *                  Overview        :This module handled all ModBus slave type messages that are received.
 *                  It generates the responses as required and triggers the transmission.
 *                  Authors         :Wenze Pan
 **************************************************************************************************************
 *                  Copyright @ As Applicable
 **************************************************************************************************************/

#ifndef MODBUSSLAVE_H_
#define MODBUSSLAVE_H_


#define MAX_READ_LENGTH                     					7
#define MAX_WRITE_LENGTH										7

// ModBus message function codes
#define MODBUS_READ_BIT_FC										0x01
#define MODBUS_READ_DESCRETE_INPUT_FC							0x02
#define MODBUS_READ_REGISTER_FC									0x03
#define MODBUS_READ_INPUT_REGISTERS_FC							0x04
#define MODBUS_WRITE_BIT_FC										0x05
#define MODBUS_WRITE_REGISTER_FC								0x06
#define MODBUS_READ_EXCEPTION_STATUS_FC							0x07
#define MODBUS_LOOPBACK_COMMAND_FC								0x08
#define MODBUS_GET_COMM_EVENT_COUNTER_FC						0x0B
#define MODBUS_GET_COMM_EVENT_LOG_FC							0x0C
#define MODBUS_CANOPEN_GENERAL_REF_REQUEST_RESPONSE_PDU_FC		0x0D
#define MODBUS_READ_DEVICE_IDENTIFICATION_FC					0x0E
#define MODBUS_WRITE_MULTIPLE_COILS_FC							0x0F
#define MODBUS_WRITE_MULTIPLE_REGISTERS_FC						0x10
#define MODBUS_REPORT_SLAVE_ID_FC								0x11
#define MODBUS_READ_FILE_RECORD_FC								0x14
#define MODBUS_WRITE_FILE_RECORD_FC								0x15
#define MODBUS_MASK_WRITE_REGISTER_FC							0x16
#define MODBUS_READWRITE_MULTIPLE_REGISTERS_FC					0x17
#define MODBUS_READ_FIFO_QUEUE_FC								0x18
#define MODBUS_ENCAPSULATED_INTERFACE_TRANSPORT_FC				0x2B

// Standard ModBus exception codes
#define ILLEGAL_FUNCTION_CODE									0x01
#define ILLEGAL_DATA_ADDRESS_CODE								0x02
#define ILLEGAL_DATA_VALUE_CODE									0x03
#define SLAVE_DEVICE_FAILURE_CODE								0x04
#define ACKNOWLEDGE_CODE										0x05
#define SLAVE_DEVICE_BUSY_CODE									0x06
#define MEMORY_PARITY_ERROR_CODE								0x08
#define GATEWAY_PATH_UNAVAILABLE_CODE							0x0A
#define GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND_CODE			0x0B

// word offset for responses including a value
#define REPLY_DATA_OFFSET										3
#define COMMAND_DATA_OFFSET										4

#define SlaveID													8

void ModbusSlave_Run();


#endif /* MODBUSSLAVE_H_ */
