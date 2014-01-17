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
/*  File Name         : queue.h                                              */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to queue data structure.                     */
/*                                                                           */
/*  List of Functions : init_list_buffer                                     */
/*                      add_list_element                                     */
/*                      remove_list_element_head                             */
/*                      remove_list_element                                  */
/*                      peek_list                                            */
/*                      next_element_list                                    */
/*                      merge_list                                           */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef QUEUE_H
#define QUEUE_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "imem_if.h"
#include "proc_if.h"
#include <linux/sched.h>

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct _q_entry_t
{
    void              *fa;          /* Pointer to the element in the queue   */
    struct _q_entry_t *tqe_next;    /* Pointer to the next element           */
    struct _q_entry_t *tqe_prev;    /* Pointer to the previous element       */
} q_entry_t;

typedef struct
{
    q_entry_t *head;
    q_entry_t *tail;
} q_head_t;

typedef struct
{
    void    *head;           /* Head Element of the List                    */
    void    *tail;           /* Tail Element of the List                    */
    UWORD16 count;           /* Number of Elements in the List              */
    UWORD16 lnk_byte_ofst;   /* Offset to Link Pointer in the List in Bytes */
    struct mutex  lock;						 /* (16bytes=4word, for detail, see BA tx dscr) */
    int		locked;          /* all single list operation need protection by zhao */
    unsigned long	owner;
    struct task_struct *tsk;	
} list_buff_t;

static inline void get_queue_mutex(list_buff_t *lbuff, unsigned long owner)
{
	if(lbuff->locked && current == lbuff->tsk){
		printk("@@@BUG! mutex has locked by %pS\n", (void *)lbuff->owner);
		dump_stack();
	}
	mutex_lock(&lbuff->lock);
	lbuff->locked = 1;
	lbuff->owner = owner;
	lbuff->tsk = current;
}

static inline void put_queue_mutex(list_buff_t *lbuff)
{
	mutex_unlock(&lbuff->lock);
	lbuff->locked = 0;
	lbuff->owner = 0;
	lbuff->tsk = NULL;
}
/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/

#define NEXT_ELEMENT_ADDR(base, ofst) (*((UWORD32 *)((UWORD8 *)base + ofst)))

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* List buffer is a queue maintained as a singly-linked link list of the   */
/* elements to be stored in the queue. It uses 4 bytes from the element    */
/* itself to store the link address of the next element. This has low      */
/* processing overhead & memory requirements. However, it assumes the      */
/* availability of the extra 4 Bytes in the element for creating the       */
/* link. These 4 bytes are specified by the user during the                */
/* initialization phase & the assigned bytes should not be modified by     */
/* the user.                                                               */
/* Note that no critical sections are currently used in this module. Hence */
/* a List Buffer should be accessed from only one context/thread.          */

/* This function initializes the List Buffer handle. The offset within the */
/* element which can be used for storing the link is specified as input.   */
INLINE void init_list_buffer(list_buff_t *lbuff, UWORD32 lnk_byte_ofst)
{
    lbuff->head          = NULL;
    lbuff->tail          = NULL;
    lbuff->count         = 0;
    lbuff->lnk_byte_ofst = lnk_byte_ofst;
    mutex_init(&lbuff->lock);    /* init the mutex and other staff by zhao 6-21 2013*/
    lbuff->locked = 0;
    lbuff->owner = 0;
    lbuff->tsk = NULL;
}

/* This function adds a new element to the end of the queue. */
INLINE void add_list_element(list_buff_t *lbuff, void *elem)
{
    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
    NEXT_ELEMENT_ADDR(elem, lbuff->lnk_byte_ofst) = 0;
    if(lbuff->tail == NULL)
        lbuff->head = elem;
    else
        NEXT_ELEMENT_ADDR(lbuff->tail, lbuff->lnk_byte_ofst) = (UWORD32)elem;

    lbuff->tail = elem;
    lbuff->count++;
	put_queue_mutex(lbuff);
}

/* This function removes the first element from the head of the queue */
INLINE void *remove_list_element_head(list_buff_t *lbuff)
{
    void *retval = NULL;

    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
	if(lbuff->head == NULL){
		put_queue_mutex(lbuff);
        	return NULL;
	}

	retval      = lbuff->head;
	lbuff->head = (void *)NEXT_ELEMENT_ADDR(lbuff->head, lbuff->lnk_byte_ofst);

	if(lbuff->head == NULL)
		lbuff->tail = NULL;
	lbuff->count--;

	put_queue_mutex(lbuff);
	return retval;
}


INLINE void *find_prev_elemt(list_buff_t *lbuff, void *elem)
{
    void *curr_el = lbuff->head;
    void *prev_el = 0;
   
    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
    while(NULL !=  curr_el)
    {
        if(curr_el == elem)
           break;
        prev_el = curr_el;
        curr_el = (void *)NEXT_ELEMENT_ADDR(curr_el, lbuff->lnk_byte_ofst);
    }
    
	put_queue_mutex(lbuff);
    return prev_el;    
}

/* This function removes a given element from the queue                      */
/* The previous element also needs to be given. The previous element ptr     */
/* will be NULL if the curr element is the head                              */
/* CAUTION: There is no check done if the element exist in the link list     */
INLINE void remove_list_element(list_buff_t *lbuff, void *prev_el,
                                    void *curr_el)
{

    void *next_el;

    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
    next_el = (void *)NEXT_ELEMENT_ADDR(curr_el, lbuff->lnk_byte_ofst);

    if(prev_el == NULL)
    {
        /* The current element is the head */
        lbuff->head = next_el;
    }
    else
    {
        NEXT_ELEMENT_ADDR(prev_el, lbuff->lnk_byte_ofst) = (UWORD32)next_el;
    }

    if(next_el == NULL)
    {
        /* The current element is the tail */
        lbuff->tail = prev_el;
    }

    lbuff->count--;

	put_queue_mutex(lbuff);
}

/* This function is used to check the head of the queue */
INLINE void *peek_list(list_buff_t *lbuff)
{
	void *rv;
	
    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
	rv = lbuff->head;
	put_queue_mutex(lbuff);
	return rv;
}

/* This function is used to get the pointer to the next element in the list  */
/* The reference is the current element                                      */
INLINE void *next_element_list(list_buff_t *lbuff, void *curr_el)
{
	void *v;
	
    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(lbuff, (unsigned long)__builtin_return_address(0));
	if(curr_el == NULL)
		v =  lbuff->head;
	else
		v =  (void *)NEXT_ELEMENT_ADDR(curr_el, lbuff->lnk_byte_ofst);
	put_queue_mutex(lbuff);
	return v;
}


/* This function merges 2 queues of the SAME type  */
/* Queue 2 is reset                                */
INLINE void merge_list(list_buff_t *q1, list_buff_t *q2)
{
    /* single list operation need protection by zhao 6-21 2013*/
	get_queue_mutex(q1, (unsigned long)__builtin_return_address(0));
	if(q1->head == NULL)
		q1->head = q2->head;
	else
		NEXT_ELEMENT_ADDR(q1->tail, q1->lnk_byte_ofst)
	    		= (UWORD32) q2->head;

	if(q2->tail != NULL)
		q1->tail = q2->tail;
	q1->count += q2->count;
	put_queue_mutex(q1);

	get_queue_mutex(q2, (unsigned long)__builtin_return_address(0));
	/* empty the 2nd Q */
	q2->head = NULL;
	q2->tail = NULL;
	q2->count = 0;
	put_queue_mutex(q2);
}

#endif /* QUEUE_H */
