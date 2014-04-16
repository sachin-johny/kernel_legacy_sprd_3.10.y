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
/*  File Name         : index_util.c                                         */
/*                                                                           */
/*  Description       : This file contains the utility functions required    */
/*                      by the MAC CE LUT interface                          */
/*                                                                           */
/*  List of Functions : get_key_index                                        */
/*                      del_key_index                                        */
/*                      find_entry                                           */
/*                      add_entry                                            */
/*                      find_entry                                           */
/*                      delete_entry                                         */
/*                      hash                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "index_util.h"
#include "core_mode_if.h"
#include <linux/slab.h>
#include <linux/sched.h>

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/
static struct mutex	stt_lock;
static int		stt_locked = 0;
static unsigned long	stt_owner = 0;
static struct task_struct	*stt_tsk = NULL;

static struct mutex	sti_lock;
static int		sti_locked = 0;
static unsigned long	sti_owner = 0;
static struct task_struct	*sti_tsk = NULL;

table_t     g_sta_table                         = {0,};
UWORD8      g_sta_index_table[ASOC_ID_BMAP_LEN] = {0};
list_buff_t g_sta_entry_list                    = {0,};

/* all single list operation need protection by zhao 6-21 2013 */
void get_stt_mutex(unsigned long owner)
{
	if(stt_locked && current == stt_tsk ){
		printk("@@@BUG! stt_mutex has locked by %luS\n",
			stt_owner);
		dump_stack();
	}
	mutex_lock(&stt_lock);
	stt_locked = 1;
	stt_owner = owner;
	stt_tsk = current;
}

void put_stt_mutex(void)
{
	mutex_unlock(&stt_lock);
	stt_locked = 0;
	stt_owner = 0;
	stt_tsk = NULL;
}

void get_sti_mutex(unsigned long owner)
{
	if(sti_locked && current == sti_tsk){
		printk("@@@BUG! sti_mutex has locked by %luS\n",
			sti_owner);
		dump_stack();
	}
	mutex_lock(&sti_lock);
	sti_locked = 1;
	sti_owner = owner;
	sti_tsk = current;
}

void put_sti_mutex(void)
{
	mutex_unlock(&sti_lock);
	sti_locked = 0;
	sti_owner = 0;
	sti_tsk = NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : hash                                                  */
/*                                                                           */
/*  Description      : This function computes the hash value for given       */
/*                     address.                                              */
/*                                                                           */
/*  Inputs           : 1) STA address                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function computes hash value of a given MAC      */
/*                     address. This value is used to access the hash table. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : UWORD32, hash value                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 hash(UWORD8* addr)
{
    UWORD32 sum = 0;
    UWORD8  i   = 0;

    for(i = 0; i < 6; i++)
        sum += addr[i];

    return sum % MAX_HASH_VALUES;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_new_sta_index                                     */
/*                                                                           */
/*  Description      : This function gets a new and unallocated sta index    */
/*                                                                           */
/*  Inputs           : 1) The Sta Address                                    */
/*                                                                           */
/*  Globals          : g_sta_index_table                                     */
/*                                                                           */
/*  Processing       : This function gets a new and unallocated sta index    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : The sta index                                         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_new_sta_index(UWORD8 *addr)
{
    table_entry_t *te = NULL;

    /* Search the entry with the given address as the key */
    te = (table_entry_t *)find_entry(addr);
    if(te == NULL)
    {
        return INVALID_ID;
    }

    /* If the element entry has a valid station index, same must be used */
    /* else a new sta index is allocated                                 */
    if(te->sta_index != INVALID_ID)
    {
        return te->sta_index;
    }
    else
    {
    	te->sta_index = alloc_sta_index();
    	TROUT_DBG4("alloc_sta_index = %d\n", te->sta_index);	//add by chengwg.
    	return te->sta_index;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : del_sta_index                                         */
/*                                                                           */
/*  Description      : This function deletes a allocated sta index           */
/*                                                                           */
/*  Inputs           : 1) Sta Index to be removed                            */
/*                                                                           */
/*  Globals          : g_sta_index_table                                     */
/*                                                                           */
/*  Processing       : This function deletes a allocated sta index           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void del_sta_index(void *val)
{
    table_entry_t *te = (table_entry_t *)val;
    if(te != NULL)
    {
        /* Reset the entry, if the index is valid */
        if(te->sta_index > 0)
        {
            UWORD8 byte = te->sta_index/8;
            UWORD8 bit  = (1 << te->sta_index % 8);
        /* all single list operation need protection by zhao 6-21 2013 */
	    get_sti_mutex((unsigned long)__builtin_return_address(0));
            g_sta_index_table[byte] &= ~bit;
	    put_sti_mutex();
        }
        te->sta_index = 0;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_sta_table                                           */
/*                                                                           */
/*  Description   : This function initializes globals for sta entry table    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : The globals g_sta_table variables should be initialized  */
/*                  to zero                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_sta_table(void)
{
    UWORD32 i = 0;
	
	TROUT_FUNC_ENTER;
	mutex_init(&stt_lock);
	mutex_init(&sti_lock);
    /* Initialize Association table */
    for(i = 0; i < MAX_HASH_VALUES; i++)
        g_sta_table[i] = 0;

    /* Reset the Key Index list */
    mem_set(g_sta_index_table, 0, ASOC_ID_BMAP_LEN);

    /* Initialize the BIT MAP so as to limit the Indices to the configured */
    /* Maximum no. of permissible stations                                 */
    if(ASOC_ID_BMAP_LEN == 1)
    {
        g_sta_index_table[ASOC_ID_BMAP_LEN - 1] =
                         (UWORD8)(1 | (0xFF << (MAX_STA_SUPPORTED + 1) ));
    }
    else
    {
        g_sta_index_table[0] =(UWORD8)1;
        if (((MAX_STA_SUPPORTED + 1)  % 8)!=0)
        {
            g_sta_index_table[ASOC_ID_BMAP_LEN - 1] =
                              (UWORD8)(0xFF << ((MAX_STA_SUPPORTED + 1)  % 8));
        }
    }

    /* Initialize the list buffer handle for the sta entry list.*/
    init_list_buffer(&g_sta_entry_list, 0);
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : alloc_sta_index                                          */
/*                                                                           */
/*  Description   : This function allocates and returns a Station index      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_sta_index_table                                        */
/*                                                                           */
/*  Processing    : Find the smallest unallocated station index using        */
/*                  g_sta_table bit map table. Mark the bit in the bit map   */
/*                  table as allocated and return the station index          */
/*                                                                           */
/*                  If there are no free association Ids, INVALID_ID is      */
/*                  returned.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Free association Id, if exists.                          */
/*                  INVALID_ID: Otherwise                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 alloc_sta_index(void)
{
    UWORD8 byte        = 0;
    UWORD8 bit         = 0;
    UWORD8 temp        = 0;

    /* all single list operation need protection by zhao 6-21 2013 */
	    get_sti_mutex((unsigned long)__builtin_return_address(0));
    /* Parse the bit map array to find out free Station Id */
    for(byte = 0; byte < ASOC_ID_BMAP_LEN; byte ++){
        temp = g_sta_index_table[byte];

        /* Note that Zero is never returned as a valid ID */
        for(bit = 0; bit < 8; bit++){
            if((bit == 0) && (byte == 0))
                continue;

            if((temp & (1 << bit)) == 0x0){
                g_sta_index_table[byte] |= (1 << bit);
		        put_sti_mutex();
                return (byte * 8 + bit);
            }
        }
    }

	    put_sti_mutex();
    /* Table is full. */
    return INVALID_ID;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : find_entry                                            */
/*                                                                           */
/*  Description      : This function searches the table for an entry having  */
/*                     the given key value.                                  */
/*                                                                           */
/*  Inputs           : 1) The key (MAC address) of the entry to be searched  */
/*                                                                           */
/*  Globals          : g_sta_table                                           */
/*                                                                           */
/*  Processing       : The hash value (val) for the given key is computed    */
/*                     using the hash function. The table entry holding the  */
/*                     given key is then searched.  If such an entry is      */
/*                     found the pointer to the entry stored in the table    */
/*                     entry is returned.                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : The element of the corresponding table entry, if an   */
/*                     entry , having a key identical to the given           */
/*                     key was found.                                        */
/*                     Zero is returned if no matching entry is found.       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void* find_entry(UWORD8* key)
{
    table_elmnt_t *tbl_ptr  = 0;
    UWORD32        val      = 0;

    /* Zero entry is fixed for Bcast/Mcast address     */
    /* If the address is BCast/MCast Address, return 0 */
	if(key[0] & BIT0)
		return 0;

	get_stt_mutex((unsigned long)__builtin_return_address(0));
	/* Calculate hash value for the key */
	val = hash(key);
	tbl_ptr = g_sta_table[val];

	while(tbl_ptr){
		if(mac_addr_cmp(tbl_ptr->key, key) == BTRUE){
			put_stt_mutex();
			return tbl_ptr->element;
		}
		tbl_ptr = tbl_ptr->next_hash_elmnt;
	}
	put_stt_mutex();

	return 0;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : add_entry                                             */
/*                                                                           */
/*  Description      : This function adds an entry to asoc table.            */
/*                                                                           */
/*  Inputs           : 1) Pointer to entry to be added to the asoc table     */
/*                     2) Pointer to the key                                 */
/*                                                                           */
/*  Globals          : g_sta_table                                           */
/*                                                                           */
/*  Processing       : A table element is created. The entry and key  values */
/*                     are set in table element and the table element is     */
/*                     inserted in the appropriate hash bucket of association*/
/*                     table.                                                */
/*                                                                           */
/*                     There is no maximum limit set for creation of assoc   */
/*                     entries. But there is a maximum limit for number of   */
/*                     associated stations, MAX_ELEMENTS.                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void add_entry(void* entry, UWORD8* key)
{
    UWORD32       val              = 0;
    table_elmnt_t *new_elm         = 0;

	/* Create a buffer to hold table element */
	new_elm = (table_elmnt_t*)mem_alloc(g_local_mem_handle,
				       sizeof(table_elmnt_t));

	if(new_elm == NULL){
		TROUT_DBG4("No Mem in func add_entry!\n");
		return;
	}

    /* all single list operation need protection by zhao 6-21 2013 */
	get_stt_mutex((unsigned long)__builtin_return_address(0));
	/* Compute the hash value */
	val = hash(key);

	/* Insert the element at head of the hash */
	memcpy(new_elm->key, key, 6);
	new_elm->next_hash_elmnt = g_sta_table[val];
	new_elm->element         = entry;
	g_sta_table[val]        = new_elm;

	/* Add the entry in the sta entry list. */
	add_list_element(&g_sta_entry_list, new_elm);
	update_entry_mode(g_sta_entry_list.count);
	put_stt_mutex();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : delete_entry                                          */
/*                                                                           */
/*  Description      : This function deletes the entry having the given key  */
/*                     value from the table. The space occupied by the entry */
/*                     is not freed here.                                    */
/*                                                                           */
/*  Inputs           : 1) The key (MAC address) of the entry to be deleted   */
/*                                                                           */
/*  Globals          : g_sta_table                                           */
/*                                                                           */
/*  Processing       : The hash value (val) for the given key is computed    */
/*                     using the hash function. The table entry holding the  */
/*                     given key is then searched.  If such an entry is      */
/*                     found the corresponding table entry is removed from   */
/*                     the hash list and freed. If no matching entry is      */
/*                     found no action is taken.                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void delete_entry(UWORD8* key)
{
    table_elmnt_t *tbl_ptr  = 0;
    table_elmnt_t *prev_ptr = 0;
    void *prev_el           = 0;
    UWORD32        val      = 0;

    /* all single list operation need protection by zhao 6-21 2013 */
	get_stt_mutex((unsigned long)__builtin_return_address(0));
	/* Calculate hash value for the key */
	val = hash(key);

	/* Initialize temporary pointers to be used in the loop below */
	tbl_ptr  = g_sta_table[val];
	prev_ptr = g_sta_table[val];

	TROUT_DBG4(" %s: called by:  ", __func__  );
	print_symbol("%s\n", (unsigned long)__builtin_return_address(0));

	/* Given the hash bucket, find out the entry that has the key */
	while(tbl_ptr){
		if(mac_addr_cmp(tbl_ptr->key, key) == BTRUE){
		    /* Got a match. Modify the next pointer of the element pointing  */
            /* to this element.                                              */
            if(tbl_ptr == g_sta_table[val])
                /* The element to be freed is the first element */
                g_sta_table[val] = tbl_ptr->next_hash_elmnt;
            else
                /* The element to be freed is not the first element */
                prev_ptr->next_hash_elmnt = tbl_ptr->next_hash_elmnt;

            /* Remove the element from the sta entry list. */
            prev_el =  find_prev_elemt(&g_sta_entry_list, (void*)tbl_ptr);
            remove_list_element(&g_sta_entry_list, prev_el, (void*)tbl_ptr);
        
            update_entry_mode(g_sta_entry_list.count);
			put_stt_mutex();

			/* Free the element referred to by this key */
			/* since in deep side of this function will re-get stt_mutex, 
			 * so, we must put_stt_mutex before it. by zhao  
			 */
			delete_element(tbl_ptr->element);
            TROUT_DBG4("delete entry: %02X:%02X:%02X:%02X:%02X:%02X\n", 
            					key[0], key[1], key[2], key[3], key[4], key[5]);
            /* Free this element */
            mem_free(g_local_mem_handle, tbl_ptr);
			TROUT_FUNC_EXIT;
            return;
        }

        prev_ptr = tbl_ptr;
        tbl_ptr = tbl_ptr->next_hash_elmnt;
    }
	put_stt_mutex();
}

#ifdef TROUT_TRACE_DBG
void print_all_entry(void)	//chengwg debug.
{
	table_elmnt_t *tbl_ptr  = g_sta_table[0];
    UWORD8 *key = NULL;
    int i = 0;

    /* all single list operation need protection by zhao 6-21 2013 */
	get_stt_mutex((unsigned long)__builtin_return_address(0));
	for(i=0; i<MAX_HASH_VALUES; i++){
		tbl_ptr = g_sta_table[i];
		if(!tbl_ptr)
			continue;

		while(tbl_ptr){
			key = tbl_ptr->key;
			printk("%d: %02x-%02x-%02x-%02x-%02x-%02x\n", 
						i, key[0], key[1], key[2], key[3], key[4], key[5]);
			tbl_ptr = tbl_ptr->next_hash_elmnt;
		}
	}
	put_stt_mutex();
}

#endif	/* TROUT_TRACE_DBG */


