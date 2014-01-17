/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : hmac_md5.c                                           */
/*                                                                           */
/*  Description       : This file contains the functions required for HMAC-  */
/*                      MD5 algorithms.                                      */
/*                                                                           */
/*  List of Functions : hmac_md5                                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "md5.h"
#include "common.h"
#include "imem_if.h"

#ifdef UTILS_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : hmac_md5                                              */
/*                                                                           */
/*  Description      : This function computes the digest for a given text    */
/*                     using the given key.                                  */
/*                                                                           */
/*  Inputs           : 1) Data                                               */
/*                     2) Data Length                                        */
/*                     3) Key                                                */
/*                     4) Key Length                                         */
/*                     5) Pointer to digest                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The digest is computed using HMAC-MD5 algorithm as    */
/*                     specified in the RFC 2104.                            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
#if 0 // caisf mod 110702
void hmac_md5(UWORD8* text, UWORD32 text_len, UWORD8* key, UWORD32 key_len,
              UWORD8* digest)
{
    UWORD32 i        = 0;
    UWORD16 curr_idx = 0;
    UWORD8  *k_ipad  = 0;//[65];
    UWORD8  *k_opad  = 0;//[65];
    UWORD8  *tk      = 0;//[16];

    md5_state_t context = {{0},};

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocate memory for local variables */
    k_ipad = (UWORD8 *)scratch_mem_alloc(65);
    if(k_ipad == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    k_opad = (UWORD8 *)scratch_mem_alloc(65);
    if(k_opad == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    tk = (UWORD8 *)scratch_mem_alloc(16);
    if(tk == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    /* If key is longer than 64 bytes reset it to key=MD5(key) */
    if (key_len > 64)
    {
        md5_state_t tctx = {{0},};

        md5_init(&tctx);
        md5_append(&tctx, key, key_len);
        md5_finish(&tctx, tk);

        key = tk;
        key_len = 16;
    }

    /* The HMAC_MD5 transform looks like:                                    */
    /* MD5(K XOR opad, MD5(K XOR ipad, text)), where K is an n byte key      */
    /*                                         ipad is byte 0x36, 64 times   */
    /*                                         opad is byte 0x5c, 64 times   */
    /*                                         text is data being protected  */

    /* The keys are stored in the pads */
    mem_set(k_ipad, 0, 65);
    mem_set(k_opad, 0, 65);
    memcpy(k_ipad, key, key_len);
    memcpy(k_opad, key, key_len);

    /* XOR key with ipad and opad values */
    for (i = 0; i < 64; i++)
    {
#if 0 //caisf mod 0702
	    k_ipad[i] ^= 0x36;
	    k_opad[i] ^= 0x5C;
#else
		k_ipad[i] = (k_ipad[i]^0x36);
		k_opad[i] = (k_ipad[i]^0x5C);
#endif
    }

    /* Inner MD5 */
    md5_init(&context);
    md5_append(&context, k_ipad, 64);
    md5_append(&context, text, text_len);
    md5_finish(&context, digest);

    /* Outer MD5 */
    md5_init(&context);
    md5_append(&context, k_opad, 64);
    md5_append(&context, digest, 16);
    md5_finish(&context, digest);

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}
#else

/* ==========================  MD5 implementation =========================== */ 
// four base functions for MD5 
#if 0
#define MD5_F1(x, y, z) (((x) & (y)) | ((~x) & (z))) 
#define MD5_F2(x, y, z) (((x) & (z)) | ((y) & (~z))) 
#define MD5_F3(x, y, z) ((x) ^ (y) ^ (z))
#define MD5_F4(x, y, z) ((y) ^ ((x) | (~z)))
#else
/* The four core functions - F1 is optimized somewhat */

/* #define F1(x, y, z) (x & y | ~x & z) */
#define F1(x, y, z) (z ^ (x & (y ^ z)))
#define F2(x, y, z) F1(z, x, y)
#define F3(x, y, z) (x ^ y ^ z)
#define F4(x, y, z) (y ^ (x | ~z))
#endif

//#define CYCLIC_LEFT_SHIFT(w, s) (((w) << (s)) | ((w) >> (32-(s))))

#if 0
#define	MD5Step(f, w, x, y,	z, data, t, s)	\
	( w	+= f(x,	y, z) +	data + t,  w = (CYCLIC_LEFT_SHIFT(w, s)) & 0xffffffff, w +=	x )
#else
/* This is the central step in the MD5 algorithm. */
#define MD5STEP(f, w, x, y, z, data, s) \
		( w += f(x, y, z) + data,  w = w<<s | w>>(32-s),  w += x )
#endif

#define MD5_MAC_LEN 16

typedef struct _MD5_CTX {
    UWORD32     Buf[4];             // buffers of four states
	UWORD8      Input[64];          // input message
	UWORD32     LenInBitCount[2];   // length counter for input message, 0 up to 64 bits	                            
}__attribute__ ((packed))       MD5_CTX;

#if 0
#ifndef BIG_ENDIAN
#define byteReverse(buf, len)   /* Nothing */
#else
void byteReverse(unsigned char *buf, unsigned longs);
void byteReverse(unsigned char *buf, unsigned longs)
{
    do {
        *(u32 *)buf = SWAP32(*(u32 *)buf);
        buf += 4;
    } while (--longs);
}
#endif
#else
#define byteReverse(buf, len)   /* Nothing */
#endif


/*
 *  Function Description:
 *      Initiate MD5 Context satisfied in RFC 1321
 *
 *  Arguments:
 *      pCtx        Pointer	to MD5 context
 *
 *  Return Value:
 *      None	    
 */
void MD5Init(MD5_CTX *pCtx)
{
    pCtx->Buf[0]=0x67452301;
    pCtx->Buf[1]=0xefcdab89;
    pCtx->Buf[2]=0x98badcfe;
    pCtx->Buf[3]=0x10325476;

	memset((u8*)pCtx->Input, 0, 64);
	
    pCtx->LenInBitCount[0]=0;
    pCtx->LenInBitCount[1]=0;
}

/*
 *  Function Description:
 *      The central algorithm of MD5, consists of four rounds and sixteen 
 *  	steps per round
 * 
 *  Arguments:
 *      Buf     Buffers of four states (output: 16 bytes)		
 * 	    Mes     Input data (input: 64 bytes) 
 *  
 *  Return Value:
 *      None
 *  	
 *  Note:
 *      Called by MD5Update or MD5Final
 */
 
static void
MD5Transform(u32 Buf[4], u32 Mes[16])
{
	u32 a, b, c, d;

	a = Buf[0];
	b = Buf[1];
	c = Buf[2];
	d = Buf[3];

	MD5STEP(F1, a, b, c, d, Mes[0] + 0xd76aa478, 7);
	MD5STEP(F1, d, a, b, c, Mes[1] + 0xe8c7b756, 12);
	MD5STEP(F1, c, d, a, b, Mes[2] + 0x242070db, 17);
	MD5STEP(F1, b, c, d, a, Mes[3] + 0xc1bdceee, 22);
	MD5STEP(F1, a, b, c, d, Mes[4] + 0xf57c0faf, 7);
	MD5STEP(F1, d, a, b, c, Mes[5] + 0x4787c62a, 12);
	MD5STEP(F1, c, d, a, b, Mes[6] + 0xa8304613, 17);
	MD5STEP(F1, b, c, d, a, Mes[7] + 0xfd469501, 22);
	MD5STEP(F1, a, b, c, d, Mes[8] + 0x698098d8, 7);
	MD5STEP(F1, d, a, b, c, Mes[9] + 0x8b44f7af, 12);
	MD5STEP(F1, c, d, a, b, Mes[10] + 0xffff5bb1, 17);
	MD5STEP(F1, b, c, d, a, Mes[11] + 0x895cd7be, 22);
	MD5STEP(F1, a, b, c, d, Mes[12] + 0x6b901122, 7);
	MD5STEP(F1, d, a, b, c, Mes[13] + 0xfd987193, 12);
	MD5STEP(F1, c, d, a, b, Mes[14] + 0xa679438e, 17);
	MD5STEP(F1, b, c, d, a, Mes[15] + 0x49b40821, 22);

	MD5STEP(F2, a, b, c, d, Mes[1] + 0xf61e2562, 5);
	MD5STEP(F2, d, a, b, c, Mes[6] + 0xc040b340, 9);
	MD5STEP(F2, c, d, a, b, Mes[11] + 0x265e5a51, 14);
	MD5STEP(F2, b, c, d, a, Mes[0] + 0xe9b6c7aa, 20);
	MD5STEP(F2, a, b, c, d, Mes[5] + 0xd62f105d, 5);
	MD5STEP(F2, d, a, b, c, Mes[10] + 0x02441453, 9);
	MD5STEP(F2, c, d, a, b, Mes[15] + 0xd8a1e681, 14);
	MD5STEP(F2, b, c, d, a, Mes[4] + 0xe7d3fbc8, 20);
	MD5STEP(F2, a, b, c, d, Mes[9] + 0x21e1cde6, 5);
	MD5STEP(F2, d, a, b, c, Mes[14] + 0xc33707d6, 9);
	MD5STEP(F2, c, d, a, b, Mes[3] + 0xf4d50d87, 14);
	MD5STEP(F2, b, c, d, a, Mes[8] + 0x455a14ed, 20);
	MD5STEP(F2, a, b, c, d, Mes[13] + 0xa9e3e905, 5);
	MD5STEP(F2, d, a, b, c, Mes[2] + 0xfcefa3f8, 9);
	MD5STEP(F2, c, d, a, b, Mes[7] + 0x676f02d9, 14);
	MD5STEP(F2, b, c, d, a, Mes[12] + 0x8d2a4c8a, 20);

	MD5STEP(F3, a, b, c, d, Mes[5] + 0xfffa3942, 4);
	MD5STEP(F3, d, a, b, c, Mes[8] + 0x8771f681, 11);
	MD5STEP(F3, c, d, a, b, Mes[11] + 0x6d9d6122, 16);
	MD5STEP(F3, b, c, d, a, Mes[14] + 0xfde5380c, 23);
	MD5STEP(F3, a, b, c, d, Mes[1] + 0xa4beea44, 4);
	MD5STEP(F3, d, a, b, c, Mes[4] + 0x4bdecfa9, 11);
	MD5STEP(F3, c, d, a, b, Mes[7] + 0xf6bb4b60, 16);
	MD5STEP(F3, b, c, d, a, Mes[10] + 0xbebfbc70, 23);
	MD5STEP(F3, a, b, c, d, Mes[13] + 0x289b7ec6, 4);
	MD5STEP(F3, d, a, b, c, Mes[0] + 0xeaa127fa, 11);
	MD5STEP(F3, c, d, a, b, Mes[3] + 0xd4ef3085, 16);
	MD5STEP(F3, b, c, d, a, Mes[6] + 0x04881d05, 23);
	MD5STEP(F3, a, b, c, d, Mes[9] + 0xd9d4d039, 4);
	MD5STEP(F3, d, a, b, c, Mes[12] + 0xe6db99e5, 11);
	MD5STEP(F3, c, d, a, b, Mes[15] + 0x1fa27cf8, 16);
	MD5STEP(F3, b, c, d, a, Mes[2] + 0xc4ac5665, 23);

	MD5STEP(F4, a, b, c, d, Mes[0] + 0xf4292244, 6);
	MD5STEP(F4, d, a, b, c, Mes[7] + 0x432aff97, 10);
	MD5STEP(F4, c, d, a, b, Mes[14] + 0xab9423a7, 15);
	MD5STEP(F4, b, c, d, a, Mes[5] + 0xfc93a039, 21);
	MD5STEP(F4, a, b, c, d, Mes[12] + 0x655b59c3, 6);
	MD5STEP(F4, d, a, b, c, Mes[3] + 0x8f0ccc92, 10);
	MD5STEP(F4, c, d, a, b, Mes[10] + 0xffeff47d, 15);
	MD5STEP(F4, b, c, d, a, Mes[1] + 0x85845dd1, 21);
	MD5STEP(F4, a, b, c, d, Mes[8] + 0x6fa87e4f, 6);
	MD5STEP(F4, d, a, b, c, Mes[15] + 0xfe2ce6e0, 10);
	MD5STEP(F4, c, d, a, b, Mes[6] + 0xa3014314, 15);
	MD5STEP(F4, b, c, d, a, Mes[13] + 0x4e0811a1, 21);
	MD5STEP(F4, a, b, c, d, Mes[4] + 0xf7537e82, 6);
	MD5STEP(F4, d, a, b, c, Mes[11] + 0xbd3af235, 10);
	MD5STEP(F4, c, d, a, b, Mes[2] + 0x2ad7d2bb, 15);
	MD5STEP(F4, b, c, d, a, Mes[9] + 0xeb86d391, 21);

	Buf[0] += a;
	Buf[1] += b;
	Buf[2] += c;
	Buf[3] += d;
}


/*
 *  Function Description:
 *      Update MD5 Context, allow of an arrary of octets as the next portion 
 *      of the message
 *      
 *  Arguments:
 *      pCtx		Pointer	to MD5 context
 * 	    pData       Pointer to input data
 *      LenInBytes  The length of input data (unit: byte)
 *
 *  Return Value:
 *      None
 *
 *  Note:
 *      Called after MD5Init or MD5Update(itself)   
 */
void MD5Update(MD5_CTX *pCtx, u8 *pData, u32 LenInBytes)
{
    
    UWORD32 TfTimes;
    UWORD32 temp;
	unsigned int i;
    
    temp = pCtx->LenInBitCount[0];

    pCtx->LenInBitCount[0] = (UWORD32) (pCtx->LenInBitCount[0] + (LenInBytes << 3));
 
    if (pCtx->LenInBitCount[0] < temp)
        pCtx->LenInBitCount[1]++;   //carry in
        
	#if 0 //dumy add for O2
    pCtx->LenInBitCount[1] += LenInBytes >> 29;
	#else
    pCtx->LenInBitCount[1] = (u32) (pCtx->LenInBitCount[1] + (LenInBytes >> 29));
	#endif
    // mod 64 bytes
    temp = (temp >> 3) & 0x3f;  
    
    // process lacks of 64-byte data 
    if (temp) 
    {
        UWORD8 *pAds = (UWORD8 *) pCtx->Input + temp;
        
        if ((temp+LenInBytes) < 64)
        {
            memcpy(pAds, (UWORD8 *)pData, LenInBytes);   
            return;
        }
        
        memcpy(pAds, (UWORD8 *)pData, 64-temp);               
        byteReverse(pCtx->Input, 16);
        MD5Transform(pCtx->Buf, (UWORD32 *)pCtx->Input);

        pData += 64-temp;
        LenInBytes -= 64-temp; 
    } // end of if (temp)
    
     
    TfTimes = (LenInBytes >> 6);

    for (i=TfTimes; i>0; i--)
    {
        memcpy(pCtx->Input, (UWORD8 *)pData, 64);
        byteReverse(pCtx->Input, 16);
        MD5Transform(pCtx->Buf, (UWORD32 *)pCtx->Input);
        pData += 64;
        LenInBytes -= 64;
    } // end of for

    // buffering lacks of 64-byte data
    if(LenInBytes)
        memcpy(pCtx->Input, (UWORD8 *)pData, LenInBytes);   
   
}


/*
 *  Function Description:
 *      Append padding bits and length of original message in the tail 
 *      The message digest has to be completed in the end  
 *  
 *  Arguments:
 *      Digest		Output of Digest-Message for MD5
 *  	pCtx        Pointer	to MD5 context
 * 	
 *  Return Value:
 *      None
 *  
 *  Note:
 *      Called after MD5Update  
 */
void MD5Final(u8 Digest[16], MD5_CTX *pCtx)
{
    UWORD8 Remainder;
    UWORD8 PadLenInBytes;
    UWORD8 *pAppend=0;
    unsigned int i;
    
    Remainder = (UWORD8)((pCtx->LenInBitCount[0] >> 3) & 0x3f);

    PadLenInBytes = (Remainder < 56) ? (56-Remainder) : (120-Remainder);
    
    pAppend = (UWORD8 *)pCtx->Input + Remainder;

    // padding bits without crossing block(64-byte based) boundary
    if (Remainder < 56)
    {
        *pAppend = 0x80;
        PadLenInBytes --;
        
        memset((UWORD8 *)pCtx->Input + Remainder+1, 0, PadLenInBytes); 
		
		// add data-length field, from low to high
       	for (i=0; i<4; i++)
        {
        	pCtx->Input[56+i] = (UWORD8)((pCtx->LenInBitCount[0] >> (i << 3)) & 0xff);
        	pCtx->Input[60+i] = (UWORD8)((pCtx->LenInBitCount[1] >> (i << 3)) & 0xff);
      	}
      	
        byteReverse(pCtx->Input, 16);
        MD5Transform(pCtx->Buf, (UWORD32 *)pCtx->Input);
    } // end of if
    
    // padding bits with crossing block(64-byte based) boundary
    else
    {
        // the first block ===
        *pAppend = 0x80;
        PadLenInBytes --;
       
        memset((u8 *)pCtx->Input + Remainder+1, 0, (64-Remainder-1)); 
        PadLenInBytes -= (64 - Remainder - 1);
        
        byteReverse(pCtx->Input, 16);
        MD5Transform(pCtx->Buf, (u32 *)pCtx->Input);
        

        // the second block ===
        memset((u8 *)pCtx->Input, 0, PadLenInBytes); 

        // add data-length field
        for (i=0; i<4; i++)
        {
        	pCtx->Input[56+i] = (UWORD8)((pCtx->LenInBitCount[0] >> (i << 3)) & 0xff);
        	pCtx->Input[60+i] = (UWORD8)((pCtx->LenInBitCount[1] >> (i << 3)) & 0xff);
      	}

        byteReverse(pCtx->Input, 16);
        MD5Transform(pCtx->Buf, (UWORD32 *)pCtx->Input);
    } // end of else


    memcpy((UWORD8 *)Digest, (UWORD32 *)pCtx->Buf, 16); // output
    byteReverse((UWORD8 *)Digest, 4);
    memset(pCtx, 0, sizeof(pCtx)); // memory free 
}




void hmac_md5(UWORD8 *data, UWORD32 data_len, UWORD8 *key, UWORD32 key_len, UWORD8 *mac)
{
	MD5_CTX	context;
    UWORD8 k_ipad[65]; /* inner padding - key XORd with ipad */
    UWORD8 k_opad[65]; /* outer padding - key XORd with opad */
    UWORD8 tk[16];
	int	i;

	//assert(key != NULL && data != NULL && mac != NULL);

	/* if key is longer	than 64	bytes reset	it to key =	MD5(key) */
	if (key_len	> 64) {
		MD5_CTX	ttcontext;

		MD5Init(&ttcontext);
		MD5Update(&ttcontext, key, key_len);
		MD5Final(tk, &ttcontext);
		//key=(u8*)ttcontext.buf;
		key	= tk;
		key_len	= 16;
		//printf("if key is longer	than 64!!!\n");
	}

	/* the HMAC_MD5	transform looks	like:
	 *
	 * MD5(K XOR opad, MD5(K XOR ipad, text))
	 *
	 * where K is an n byte	key
	 * ipad	is the byte	0x36 repeated 64 times
	 * opad	is the byte	0x5c repeated 64 times
	 * and text	is the data	being protected	*/

	/* start out by	storing	key	in pads	*/
	memset(k_ipad, 0, sizeof(k_ipad));
	memset(k_opad, 0, sizeof(k_opad));
	//assert(key_len < sizeof(k_ipad));
	memcpy(k_ipad, key,	key_len);
	memcpy(k_opad, key,	key_len);

	/* XOR key with	ipad and opad values */
	for	(i = 0;	i <	64;	i++) {
		k_ipad[i] = k_ipad[i] ^ 0x36;
		k_opad[i] = k_opad[i] ^ 0x5c;
	}
	
	/* perform inner MD5 */
	MD5Init(&context);					 /*	init context for 1st pass */

	MD5Update(&context,	k_ipad,	64);	 /*	start with inner pad */

	MD5Update(&context,	data, data_len); /*	then text of datagram */
	
	MD5Final(mac, &context);			 /*	finish up 1st pass */

	/* perform outer MD5 */
	MD5Init(&context);					 /*	init context for 2nd pass */
	MD5Update(&context,	k_opad,	64);	 /*	start with outer pad */
	MD5Update(&context,	mac, 16);		 /*	then results of	1st	hash */
	MD5Final(mac, &context);			 /*	finish up 2nd pass */
}
#endif

#endif /* UTILS_11I */
#endif /* MAC_802_11I */
