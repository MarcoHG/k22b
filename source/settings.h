/*
 * inputSettings.h
 *
 *  Created on: Apr 6, 2019
 *      Author: marco
 */

#ifndef INPUTSETTINGS_H_
#define INPUTSETTINGS_H_

#define TOKEN_LEN					20
#define NBR_R_SETTINGS		2		/* Read only		*/


typedef	char abToken[TOKEN_LEN];

typedef enum parse_result_t	{
	PARSE_INVALID_INPUT,	// Invalid data input: not enough, unknown chars
	PARSE_UNKOWN_TOKEN,		// Token not found
	PARSE_ERR,
	PARSE_LIM_ERR,				// Value out of limits
	PARSE_ASSIGN,
	PARSE_READ
} parse_result_t;

typedef enum mode_t	{
	MODE_RUN =0,
	MODE_MANUAL,
	MODE_AMPLIFIER

} mode_t;

typedef struct __attribute__((packed)) inputSettings
{
	const char *pToken;
	int32_t	min;
	int32_t	max;
	int32_t	*pData;
} inputSet_t;

typedef enum ctrl_mode_t	{
	RUN_MODE,
	MANUAL_MODE,
	AMPLIFIER_MODE
} ctrl_mode_t;

/*!
 * Define the Control's settings
 */
typedef struct  __attribute__((packed)) parameter_data
{
  int32_t checkSpeed;
  int32_t closingSpeed;
  int32_t apertureSpeed;
  int32_t waitAtOpenMsec;
  int32_t checkPercent;
  int32_t partialPercent;
  int32_t currentLimit;
  int32_t regenLimit;
  int32_t acel;
  int32_t decel;
  int32_t checkSpeedDir;
  // spares
  int32_t 	set1;
  int32_t 	set2;
  uint32_t	crc;

} stPrmData;

parse_result_t parseInStr(inputSet_t *pSett, char *inStr, char *outStr, uint8_t *pIndex);

extern	stPrmData 	gData;
extern 	inputSet_t 	gUsrInput[];
extern 	uint16_t 		gNbrUsrInput;

#endif /* INPUTSETTINGS_H_ */
