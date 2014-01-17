#ifndef WAPI_SMS4_H
#define WAPI_SMS4_H

#include "common.h"

/** \@{
 * @ingroup crypto
 */
/**
 * @brief SMS4
 *
 * @par Description
 *
 * @param[in]   iv          : Initialization value/integrity vector
 * @param[in]   key         : key
 * @param[in]   input       : input text
 * @param[in]   length      : Length of text
 * @param[inout]output      : Computed sms4 output
 *
 * @return
 *   void
 */
extern void WapiCryptoSms4(UWORD8 *iv, UWORD8 *key, UWORD8 *input, UWORD16 length, UWORD8 *output);

/** \@{
 * @ingroup crypto
 */
/**
 * @brief SMS4 MIC
 *
 * @par Description
 *
 * @param[in]   iv          : Initialization value/integrity vector
 * @param[in]   key         : key
 * @param[in]   header      : header padded to 16 byte boundary
 * @param[in]   headerLength: Length of header
 * @param[in]   input       : input data
 * @param[in]   length      : Length of data
 * @param[inout]output      : Computed MIC
 *
 * @return
 *   void
 */
extern void WapiCryptoSms4Mic(UWORD8 *iv, UWORD8 *Key, UWORD8 *header, UWORD16 headerLength,
                             UWORD8 *input, UWORD16 dataLength, UWORD8 *output);

/** \@}
 */



#endif
