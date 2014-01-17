
//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

/*1，消息鉴别算法参考实现*/
#include <string.h>
#include "common.h"

#include "csl_linux.h"


#define SHA256_BLOCK_SIZE 64
#define SHA256_DIGEST_SIZE 32
typedef unsigned char byte;

typedef enum{
    FALSE = 0, 
    TRUE = 1
}CsrBool;

#if 0
typedef struct contxt
{
	byte *buff;
	unsigned length;
} CONTX;
int sha256(CONTX*,int,byte*);


/*
a) unsigned char *text 表示进行HMAC 运算的文本；
b) unsigned text_len 表示进行HMAC 运算的文本的长度（八位位组数）；
c) byte *key 表示进行HMAC 运算的密钥；
d) unsigned key_len 表示进行HMAC 运算的密钥的长度（八位位组数）；
e) byte *digest 表示进行HMAC 运算输出的摘要；
f) unsigned digest_length 表示进行HMAC 运算，要求输出的摘要长度（八位位组数），必须小于或等于sha-256 杂凑算法可以输出的摘要长度（八位位组数）。
g) 返回值，非0 表示实际输出的摘要长度（八位位组数），0 表示失败。
*/
int hmac_sha256(unsigned char *text, int text_len, 
					byte *key, unsigned key_len, 
					byte *digest,unsigned digest_length)

{
	byte real_key[SHA256_BLOCK_SIZE];
	byte ipad[SHA256_BLOCK_SIZE];
	byte opad[SHA256_BLOCK_SIZE];
	byte temp_digest1[SHA256_DIGEST_SIZE];
	byte temp_digest2[SHA256_DIGEST_SIZE];
	CONTX input_data[2];
	unsigned i;
	if (digest_length>SHA256_DIGEST_SIZE)
		return 0;
	for(i=0;i< SHA256_BLOCK_SIZE;i++){
		real_key[i]=0;
		ipad[i]=0x36;
		opad[i]=0x5c;
	}
	/* if key_len is larger than hash block size, key is hashed first to make its length is equal hash block size */
	if(key_len> SHA256_BLOCK_SIZE){
		input_data[0].buff=key;
		input_data[0].length=key_len;
		//SHA256(input_data,1,real_key);
		key_len= SHA256_BLOCK_SIZE;
	}
	else
		memcpy(real_key,key,key_len);
	for(i=0;i< SHA256_BLOCK_SIZE;i++){
		ipad[i]^=real_key[i];
		opad[i]^=real_key[i];
	}
	/*sha256(Key xor ipad,text)=temp_digest1 */
	input_data[0].buff=ipad;
	input_data[0].length= SHA256_BLOCK_SIZE;
	input_data[1].buff=text;
	input_data[1].length=text_len;
	//SHA256(input_data,2,temp_digest1);
	/*sha256(Key xor opad,temp_digest1)=temp_digest2 */
	input_data[0].buff=opad;
	input_data[0].length= SHA256_BLOCK_SIZE;
	input_data[1].buff=temp_digest1;
	input_data[1].length =SHA256_DIGEST_SIZE;
	//SHA256(input_data,2,temp_digest2);
	/*output the digest of required length */
	memcpy(digest,temp_digest2,digest_length);
	return digest_length;
}

/*2，密钥导出算法参考实现*/
//#define SHA256_BLOCK_SIZE 64
//#define SHA256_DIGEST_SIZE 32
//int hmac_sha256(unsigned char, int, byte, unsigned , byte *, unsigned);

/*
a) byte *text 表示密钥导出算法的输入文本；
b) unsigned text_len 表示输入文本的长度（八位位组数）；
c) byte *key 表示密钥导出算法的输入密钥；
d) unsigned key_len 表示输入密钥的长度（八位位组数）；
e) byte *output 表示密钥导出算法的输出；
f) unsigned length 表示密钥导出算法的输出的长度（八位位组数）。
*/
void KD_hmac_sha256(byte *text, unsigned text_len, byte *key, unsigned key_len, byte *output, unsigned
length)
{
	int i;
	for(i=0;length/SHA256_DIGEST_SIZE;i++,length-=SHA256_DIGEST_SIZE){
		hmac_sha256(text,text_len,key,key_len,&output[i*SHA256_DIGEST_SIZE],SHA256_DIGEST_SIZE);
		text=&output[i*SHA256_DIGEST_SIZE];
		text_len=SHA256_DIGEST_SIZE;
	}
	if(length>0)
		hmac_sha256(text,text_len,key,key_len,&output[i*SHA256_DIGEST_SIZE],length);
}
#endif
/*3，SMS4算法参考实现*/
/* PRIVATE CONSTANT DEFINITIONS *********************************************/
#define BYTES_PER_WORD  4
#define BYTE_LEN        8
#define WORD_LEN        (BYTE_LEN * BYTES_PER_WORD)
#define TEXT_LEN        128
#define MK_LEN          (TEXT_LEN / WORD_LEN)
#define RK_LEN          32
#define TEXT_BYTES      (TEXT_LEN / BYTE_LEN)

#define CK_INCREMENT    7
#define KEY_MULTIPLIER  0x80040100
#define TEXT_MULTIPLIER 0xa0202080
#define FK_PARAMETER_0  0xa3b1bac6
#define FK_PARAMETER_1  0x56aa3350
#define FK_PARAMETER_2  0x677d9197
#define FK_PARAMETER_3  0xb27022dc

static const UWORD8 S_Box[] = {
/*        0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f */
/*====================================================================================================*/
/* 0 */0xd6, 0x90, 0xe9, 0xfe, 0xcc, 0xe1, 0x3d, 0xb7, 0x16, 0xb6, 0x14, 0xc2, 0x28, 0xfb, 0x2c, 0x05,
/* 1 */0x2b, 0x67, 0x9a, 0x76, 0x2a, 0xbe, 0x04, 0xc3, 0xaa, 0x44, 0x13, 0x26, 0x49, 0x86, 0x06, 0x99,
/* 2 */0x9c, 0x42, 0x50, 0xf4, 0x91, 0xef, 0x98, 0x7a, 0x33, 0x54, 0x0b, 0x43, 0xed, 0xcf, 0xac, 0x62,
/* 3 */0xe4, 0xb3, 0x1c, 0xa9, 0xc9, 0x08, 0xe8, 0x95, 0x80, 0xdf, 0x94, 0xfa, 0x75, 0x8f, 0x3f, 0xa6,
/* 4 */0x47, 0x07, 0xa7, 0xfc, 0xf3, 0x73, 0x17, 0xba, 0x83, 0x59, 0x3c, 0x19, 0xe6, 0x85, 0x4f, 0xa8,
/* 5 */0x68, 0x6b, 0x81, 0xb2, 0x71, 0x64, 0xda, 0x8b, 0xf8, 0xeb, 0x0f, 0x4b, 0x70, 0x56, 0x9d, 0x35,
/* 6 */0x1e, 0x24, 0x0e, 0x5e, 0x63, 0x58, 0xd1, 0xa2, 0x25, 0x22, 0x7c, 0x3b, 0x01, 0x21, 0x78, 0x87,
/* 7 */0xd4, 0x00, 0x46, 0x57, 0x9f, 0xd3, 0x27, 0x52, 0x4c, 0x36, 0x02, 0xe7, 0xa0, 0xc4, 0xc8, 0x9e,
/* 8 */0xea, 0xbf, 0x8a, 0xd2, 0x40, 0xc7, 0x38, 0xb5, 0xa3, 0xf7, 0xf2, 0xce, 0xf9, 0x61, 0x15, 0xa1,
/* 9 */0xe0, 0xae, 0x5d, 0xa4, 0x9b, 0x34, 0x1a, 0x55, 0xad, 0x93, 0x32, 0x30, 0xf5, 0x8c, 0xb1, 0xe3,
/* a */0x1d, 0xf6, 0xe2, 0x2e, 0x82, 0x66, 0xca, 0x60, 0xc0, 0x29, 0x23, 0xab, 0x0d, 0x53, 0x4e, 0x6f,
/* b */0xd5, 0xdb, 0x37, 0x45, 0xde, 0xfd, 0x8e, 0x2f, 0x03, 0xff, 0x6a, 0x72, 0x6d, 0x6c, 0x5b, 0x51,
/* c */0x8d, 0x1b, 0xaf, 0x92, 0xbb, 0xdd, 0xbc, 0x7f, 0x11, 0xd9, 0x5c, 0x41, 0x1f, 0x10, 0x5a, 0xd8,
/* d */0x0a, 0xc1, 0x31, 0x88, 0xa5, 0xcd, 0x7b, 0xbd, 0x2d, 0x74, 0xd0, 0x12, 0xb8, 0xe5, 0xb4, 0xb0,
/* e */0x89, 0x69, 0x97, 0x4a, 0x0c, 0x96, 0x77, 0x7e, 0x65, 0xb9, 0xf1, 0x09, 0xc5, 0x6e, 0xc6, 0x84,
/* f */0x18, 0xf0, 0x7d, 0xec, 0x3a, 0xdc, 0x4d, 0x20, 0x79, 0xee, 0x5f, 0x3e, 0xd7, 0xcb, 0x39, 0x48 };

static const UWORD32 FK_Parameter[] = { FK_PARAMETER_0, FK_PARAMETER_1, FK_PARAMETER_2, FK_PARAMETER_3 };


static const UWORD8 S_XState[] = {
/*   0x0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f */
/*        0  1  1  2  1  2  2  3  1  2  2  3  2  3  3  4 */
/*====================================================================================================*/
/*0x0 0 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0x00-0x0F */
/*  1 1 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0x10-0x1F */
/*  2 1 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0x20-0x2F */
/*  3 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0x30-0x3F */
/*  4 1 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0x40-0x4F */
/*  5 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0x50-0x5F */
/*  6 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0x60-0x6F */
/*  7 3 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0x70-0x7F */
/*  8 1 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0x80-0x8F */
/*  9 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0x90-0x9F */
/*  a 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0xA0-0xAF */
/*  b 3 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0xB0-0xBF */
/*  c 2 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,   /* 0xC0-0xCF */
/*  d 3 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0xD0-0xDF */
/*  e 3 */1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,   /* 0xE0-0xEF */
/*  f 4 */0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0    /* 0xF0-0xFF */
};

static const UWORD32 g_NextInputTable[RK_LEN] = 
{
    0x00070e15, 0x1c232a31, 0x383f464d, 0x545b6269,
    0x70777e85, 0x8c939aa1, 0xa8afb6bd, 0xc4cbd2d9,
    0xe0e7eef5, 0xfc030a11, 0x181f262d, 0x343b4249,
    0x50575e65, 0x6c737a81, 0x888f969d, 0xa4abb2b9,
    0xc0c7ced5, 0xdce3eaf1, 0xf8ff060d, 0x141b2229,
    0x30373e45, 0x4c535a61, 0x686f767d, 0x848b9299,
    0xa0a7aeb5, 0xbcc3cad1, 0xd8dfe6ed, 0xf4fb0209,
    0x10171e25, 0x2c333a41, 0x484f565d, 0x646b7279
};

static const UWORD32 CipherDataIdx[MK_LEN][MK_LEN] = 
{
    {3, 2, 1, 0},
    {0, 3, 2, 1},
    {1, 0 ,3, 2},
    {2, 1, 0, 3}
};

#define PARITY_MACRO(Value) (S_XState[(Value) >> 24] ^ S_XState[((Value) >> 16) & 0xFF] ^ S_XState[((Value) >> 8) & 0xFF] ^ S_XState[(Value) & 0xFF])
#define XOR_MACRO(A, B) ((A) ^ (B))
#define L_TRANSFORM_MACRO(Word, Key) MultiplyCircular(Word, Key ? KEY_MULTIPLIER : TEXT_MULTIPLIER )

/* PRIVATE VARIABLE DEFINITIONS *********************************************/

/* PRIVATE FUNCTION PROTOTYPES **********************************************/

/* PRIVATE FUNCTION DEFINITIONS *********************************************/

static UWORD32 T_Transform(UWORD32 Word)
{
    UWORD32 j;
    UWORD32 New_Word;
	int offset = 0;

    New_Word = 0;
    for (j = 0; j < MK_LEN; j++)
    {
        New_Word = (New_Word << BYTE_LEN);
		offset   = ((UWORD32)(Word >> (WORD_LEN - BYTE_LEN))) & ((UWORD32)((1 << BYTE_LEN) - 1));
		New_Word = New_Word | (UWORD32)S_Box[offset];
		Word = (Word << BYTE_LEN);
    }
    return (New_Word);
}

static UWORD32 MultiplyCircular(UWORD32 Word, UWORD32 Basis)
{
    UWORD32 New_Word;
    UWORD32 i;

    New_Word = 0;

    for (i = 0; i < WORD_LEN; i++)
    {
		New_Word = (New_Word << 1) | PARITY_MACRO(Word & Basis);

        Basis     = (Basis >> 1) | ((Basis & 1) << (WORD_LEN - 1));
    }
    return (New_Word);
}



static UWORD32 Iterate(CsrBool Key, UWORD32 Next_Input, UWORD32 *Cipher_Text, UWORD32 curIdx)
{
    UWORD32 New_State;

    New_State = Next_Input;
    New_State = XOR_MACRO(New_State, Cipher_Text[CipherDataIdx[curIdx][0]]);
    New_State = XOR_MACRO(New_State, Cipher_Text[CipherDataIdx[curIdx][1]]);
    New_State = XOR_MACRO(New_State, Cipher_Text[CipherDataIdx[curIdx][2]]); 
    New_State = L_TRANSFORM_MACRO(T_Transform(New_State), Key);
    New_State = XOR_MACRO(New_State, Cipher_Text[CipherDataIdx[curIdx][3]]);

    Cipher_Text[curIdx] = New_State;

    return (New_State);
}

static void CalculateEnKey(UWORD8 *Key, UWORD32 *Key_Store)
{
    UWORD32 Cipher_Text[MK_LEN];

    UWORD32 Next, i, j, Next_Input;

    for (j = 0; j < MK_LEN; j++)
    {
        Next = 0;
        for (i = 0; i < BYTES_PER_WORD; i++)
        {
		    Next = (Next << BYTE_LEN);
		    Next = Next | Key[(j <<2) + i];
        }

        Cipher_Text[j] = XOR_MACRO(Next, FK_Parameter[j]);
    }

    for (i = 0; i < RK_LEN; i++)
    {
        Next_Input = g_NextInputTable[i];

        Key_Store[i] = Iterate(TRUE, Next_Input, Cipher_Text, i & (MK_LEN - 1));
    }
}

static void SMS4_Run(UWORD32 *Key_Store, UWORD8 *PlainText, UWORD8 *CipherText)
{
    UWORD32 i, j;
    UWORD32 Next;
    UWORD32 Next_Input;
    UWORD32 Plain_Text[MK_LEN];

    for (j = 0; j < MK_LEN; j++)
    {
        Next = 0;
        for (i = 0; i < BYTES_PER_WORD; i++)
        {
			Next = (Next << BYTE_LEN);
            Next = Next | PlainText[(j<<2) + i];
        }
        Plain_Text[j] = Next;
    }

    for (i = 0; i < RK_LEN; i++)
    {
        Next_Input = Key_Store[i];
        (void)Iterate(FALSE, Next_Input, Plain_Text, i & (MK_LEN - 1));
    }

    for (j = 0; j < MK_LEN; j++)
    {
        Next = Plain_Text[(MK_LEN - 1) - j];
        for (i = 0; i < BYTES_PER_WORD; i++)
        {
            CipherText[(j << 2) + i] = (UWORD8)((Next >> (WORD_LEN - BYTE_LEN)) & ((1 << BYTE_LEN) - 1));
			Next = (Next << BYTE_LEN);
        }
    }
}

/* PUBLIC FUNCTION DEFINITIONS **********************************************/
/*
 * Description:
 * See description in csr_sms4.h
 */
/*---------------------------------------------------------------------------*/
void WapiCryptoSms4(UWORD8 *iv, UWORD8 *key, UWORD8 *input, UWORD32 length, UWORD8 *output)
{
    UWORD32 i;
    UWORD8 sms4Output[TEXT_BYTES];
	UWORD8 tmp_data[TEXT_BYTES];

    UWORD32 Key_Store[RK_LEN];

    UWORD32 j = 0;
    UWORD8 * p[2];

    p[0] = sms4Output;
    p[1] = tmp_data;

    memcpy(tmp_data, iv, TEXT_BYTES);

    CalculateEnKey(key, Key_Store);

    for (i = 0; i < length; i++)
    {
        if ((i & (TEXT_BYTES - 1)) == 0)
        {
            SMS4_Run(Key_Store, p[1-j], p[j]);

            j = 1 - j;
        }

        output[i] = input[i] ^ p[1-j][i & (TEXT_BYTES - 1)];
    }
}

/*
 * Description:
 * See description in csr_sms4.h
 */
/*---------------------------------------------------------------------------*/
void WapiCryptoSms4Mic(UWORD8 *iv, UWORD8 *key, UWORD8 *header, UWORD32 headerLength,
                      UWORD8 *input, UWORD32 dataLength, UWORD8 *mic)
{
    UWORD32 i, j = 0, totalLength;
    UWORD8 sms4Output[TEXT_BYTES], sms4Input[TEXT_BYTES];
	UWORD32 tmp_headerLength = 0;
	UWORD32 tmp_dataLength = 0;

	UWORD32 header_cnt  = 0 ;
	UWORD32 header0_cnt = 0;
	UWORD32 data_cnt = 0;
	UWORD32 data0_cnt = 0;

    UWORD32 Key_Store[RK_LEN];

    memcpy(sms4Input, iv, TEXT_BYTES);

    totalLength = headerLength + dataLength;
	tmp_headerLength = ((headerLength & (TEXT_BYTES-1)) == 0) ? 0 : (TEXT_BYTES - (headerLength & (TEXT_BYTES-1)));
	tmp_dataLength   = ((dataLength & (TEXT_BYTES-1))   == 0) ? 0 : (TEXT_BYTES - (dataLength & (TEXT_BYTES-1)));

    totalLength += tmp_headerLength;
	totalLength += tmp_dataLength;

    CalculateEnKey(key, Key_Store);

    for (i = 0; i < totalLength; i++)
    {
        if ((i & (TEXT_BYTES-1)) == 0)
        {
            SMS4_Run(Key_Store, sms4Input, sms4Output);
        }

        if ((dataLength == 0) && (headerLength == 0))
        {
            sms4Input[i & (TEXT_BYTES-1)] = 0 ^ sms4Output[i & (TEXT_BYTES-1)]; /* lint !e644 */

			data0_cnt++;
        }
        else if ( (headerLength == 0) && (tmp_headerLength == 0) )
        {
			sms4Input[i & (TEXT_BYTES-1)] = input[j] ^ sms4Output[i & (TEXT_BYTES-1)];
			j++;
            dataLength--;

			data_cnt++;
        }
		else if( headerLength == 0 )
		{
			sms4Input[i & (TEXT_BYTES-1)] = 0 ^ sms4Output[i & (TEXT_BYTES-1)]; /* lint !e644 */
			tmp_headerLength--;

			header0_cnt++;
		}
        else
        {
            sms4Input[i & (TEXT_BYTES-1)] = header[i] ^ sms4Output[i & (TEXT_BYTES-1)];
            headerLength--;

			header_cnt++;
        }
    }

    SMS4_Run(Key_Store, sms4Input, mic);
}

#if 0
UWORD8 output_data1[72] = {0};
UWORD8 output_data2[72+16] = {0};

UWORD8 input_head[] = 
{
0x88,0x41,
//0x41,0x88,

0x58,0x66,0xba,0x6b,0x11,0x61,	
0x00,0x3d,0x5e,0x2d,0xe6,0x41,

//0x30,0x00,
0x00,0x00,

0x33,0x33,0xff,0x2d,0xe6,0x41,
0x00,0x00,0x00,0x00,0x00,0x00,

0x00,0x00,

0x00,

0x00,

0x00,0x48,
//0x48,0x00,

//0x3a,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c
//0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x3a
};

UWORD16 head_len = 34;//16;

UWORD8 input_data1[72+16] = 
{
0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 
0x86, 0xDD, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3A, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0x2D, 0xE6, 0x41, 0x87, 0x00, 0x50, 0x0C, 0x00, 0x00,
0x00, 0x00, 0xFE, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x3D, 0x5E, 0xFF, 0xFE, 0x2D,
0xE6, 0x41
};

UWORD8 input_data2[] = 
{
0xD1, 0x30, 0x2B, 0x35, 0x2D, 0xC5,
0xAD, 0x40, 0xE6, 0xDA, 0x7D, 0xE9, 0x88, 0x30, 0x64, 0x8F, 0xAA, 0xB0, 0xBE, 0xEE, 0x73, 0xD6, 
0x68, 0x95, 0x47, 0xAD, 0xAA, 0xDF, 0x47, 0x11, 0xAF, 0xC0, 0xAD, 0xA6, 0xFA, 0x11, 0x41, 0xD1, 
0xF8, 0x6F, 0x72, 0x6E, 0xE4, 0xAF, 0xB5, 0xEA, 0x2D, 0xCF, 0x7B, 0x3C, 0x0A, 0xA0, 0x18, 0x1E, 
0xBD, 0x5F, 0xE4, 0x08, 0x4E, 0x21, 0xC2, 0x56, 0x53, 0x29, 0xDA, 0xBE, 0x90, 0xEC, 0xAE, 0xF1, 
0xA2, 0x4D
};

UWORD16 ptk_len = 72;

UWORD8 input_pn[] =
{0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x3a};
//{0x3a,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c,0x36,0x5c};

UWORD8 input_key[] =
//{0x95,0xde,0x27,0x7e,0xb4,0x8d,0x38,0xb1,0x4c,0x61,0x65,0x20,0xfb,0x66,0x32,0x3c};
{0x3c,0x32,0x66,0xfb,0x20,0x65,0x61,0x4c,0xb1,0x38,0x8d,0xb4,0x7e,0x27,0xde,0x95};

UWORD8 input_mic_key[] =
{0x74,0xe0,0x90,0x13,0x0a,0x6d,0x26,0xbd,0x5e,0xda,0xf1,0xa3,0x82,0x95,0x1a,0x9e};
//{0x9e,0x1a,0x95,0x82,0xa3,0xf1,0xda,0x5e,0xbd,0x26,0x6d,0x0a,0x13,0x90,0xe0,0x74};

UWORD8 ptk_mic[16] = {0};

void main(void)
{
	int i = 0;

	CsrCryptoSms4Mic(input_pn,input_mic_key,input_head,head_len,
                     input_data1,ptk_len,ptk_mic);

	memcpy(&input_data1[72],ptk_mic,16);

	CsrCryptoSms4(input_pn,input_key,
	              input_data1,ptk_len + 16,
	              output_data2);


#if 1
	for( i = 0 ; i < ptk_len + 16 ; i++)
	{
		printf("%02x ",output_data2[i]);
		if( (i & 0x0f) == 0x0f )
		{
			printf("\n");
		}
	}
	printf("\n=======================\n");
#endif

}
#endif

#endif
