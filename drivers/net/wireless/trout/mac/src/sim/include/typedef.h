#ifndef _TYPE_DEF_H
#define _TYPE_DEF_H

#ifndef UWORD32
typedef unsigned int UWORD32;
#undef UWORD16
typedef unsigned short int UWORD16;
#undef UWORD8
typedef unsigned char UWORD8;
#undef UWORD64
typedef unsigned long long UWORD64;
#endif

#ifndef WORD32
typedef signed int WORD32;
#undef WORD16
typedef signed short int WORD16;
#undef WORD8
typedef signed char WORD8;
#undef WORD64
typedef signed long long WORD64;
#endif

#ifndef NULL
#define NULL    (void *)0
#endif

#ifndef BOOL
typedef enum{
    FALSE = 0, 
    TRUE = 1
}BOOL;
#endif

#ifndef DV_TRUE
#define DV_TRUE 1
#endif

#ifndef DV_FALSE
#define DV_FALSE 0
#endif

static inline int is_multiple_of_4(unsigned long val)
{
	return !(val & 0x03);
}
#endif  /* _TYPE_DEF_H */

