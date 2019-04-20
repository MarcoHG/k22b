/*
 ============================================================================
 Name        : settings.c
 Author      : mh
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>	// strcmp
#include <stdint.h>	// std types
#include <ctype.h>	// toupper
#include <stdlib.h>     /* strtol */
#include "settings.h"

//uint16_t	nbr_settings;

// static abToken tokens[NBR_TOKEN_CLI];	// Number of tokens per command line input

/*!
 * \brief Parse the token in input string and perform operation in the indicated table
 *
 * For a single token, get value the value in outStr
 * For assignment, token=value, set the associated variable to the vaalue
 *
 * 	\returns the operation result
 *
 */

parse_result_t parseInStr(inputSet_t *pSets, char *inStr, char *outStr, uint8_t *pIndex)
{
	parse_result_t result = PARSE_INVALID_INPUT;


	int i;
	if(strlen(inStr)==0)
		return result;

	for(i=0; i<strlen(inStr); ++i)		// upper case
		inStr[i]= toupper(inStr[i]);

	// Separate input string into tokens
	uint8_t bAssign = strchr(inStr,'=') != NULL ? 1 :0;	// is there any assignment operation?
	char *pTok = strtok (inStr," =\r\n\t");
	uint8_t nbrTokens=0;
	abToken tokenStr, valueStr;
	while (pTok != NULL && nbrTokens < 2)	// ignore extra tokens
	{
		if(nbrTokens ==0)
			strcpy(tokenStr, pTok);
		else
			strcpy(valueStr, pTok);
		pTok = strtok (NULL, " =\r\n\t");
		++nbrTokens;
	}

	// Look for a match on the gSetting list
	for(i=0; i< gNbrUsrInput; ++i)
	if(strcmp(tokenStr, pSets[i].pToken)==0)	// look for a match using token as the key in the list
	{
	  if(bAssign)
	  {
	  	int32_t	value = strtol(valueStr, NULL, 0);
	  	if(value >= pSets[i].min && value <= pSets[i].max)
	  	{
	  		*pSets[i].pData = value;
	  		result = PARSE_ASSIGN;
	  		*pIndex =i;	// Parameter index
	  	}
	  	else
	  		result = PARSE_LIM_ERR;

	  }
	  else
	  if(i < (gNbrUsrInput + NBR_R_SETTINGS) )
	  {
	  	sprintf(outStr,"%d",*pSets[i].pData);
	  	result = PARSE_READ;
	  	*pIndex =i;	// Parameter index
	  }
	  else
	  {

	  }
	  break;	// we are done
	}
		// Return "Token not found"
	if(i == gNbrUsrInput)
		result = PARSE_UNKOWN_TOKEN;
	return result;
}


