/*
 * pflash.h
 *
 *  Created on: Apr 19, 2019
 *      Author: marco
 */

#ifndef PFLASH_H_
#define PFLASH_H_



bool pflash_init(void);
bool	pflash_write(uint32_t *pData, uint16_t nbrWords);
bool	pflash_read(uint32_t *pData, uint16_t nbrWords);

#endif /* PFLASH_H_ */
