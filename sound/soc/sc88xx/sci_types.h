#ifndef SCI_TYPES_H
#define SCI_TYPES_H

typedef 	unsigned char 		BOOLEAN;



#define 	LOCAL		static
#define	PUBLIC		
#define 	SCI_TRUE	1
#define	SCI_FALSE	0

#define 	SCI_ASSERT(condition) 	BUG_ON(!(condition))

#define	SCI_NULL	0

#define PNULL  ((void *)0)

#define		CONST		const

#define REG32(addr)		(*(volatile unsigned int *) (addr))



#endif
