//#include "trout2_interface.h"
#include <linux/delay.h>

#define rMAC_PHY_REG_ACCESS_CON 0x8638
#define rMAC_PHY_REG_RW_DATA    0x863c
#define  REGBIT0          0x01
#define BIT0    0X01
#define BIT1    0X02

typedef unsigned int UWORD32;
typedef unsigned char UWORD8;

extern unsigned int host_read_trout_reg(unsigned int reg_addr);
extern unsigned int host_write_trout_reg(unsigned int val, unsigned int reg_addr);

extern void get_phy_mutex(void);
extern void put_phy_mutex(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : read_dot11_phy_reg                                       */
/*                                                                           */
/*  Description   : This function reads the Ittiam PHY register.             */
/*                                                                           */
/*  Inputs        : 1) Address of register                                   */
/*                  2) Pointer to value to be updated                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the lock bit and then reads the     */
/*                  PHY register by writing the address to the serial        */
/*                  control register, polling the status bit and then        */
/*                  reading the register.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void read_dot11_phy_reg(UWORD8 ra, UWORD32 *rd)
{
    UWORD32 val = 0;
    UWORD32 delay = 0;

    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON) & REGBIT0) == REGBIT0)
    {
        mdelay(1);
        delay++;

       /* Wait for sometime for the CE-LUT update operation to complete */
        if(delay > 10)
        {
	printk("read phy reg is timeout\n");
        break;
        }
    }

    val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
    val |= BIT1;              /* Read/Write bit set to 1 for read */
    val |= BIT0;              /* Access bit set to 1 */

    //chenq mod
    //rMAC_PHY_REG_ACCESS_CON = convert_to_le(val);
    host_write_trout_reg( val, (UWORD32)rMAC_PHY_REG_ACCESS_CON );


    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON)
                & REGBIT0) == REGBIT0)
    {
      mdelay(1);
      delay++;
      if(delay > 10)
     {
	printk("read phy reg is timeout\n");
        break;
        }

    }
    *rd = host_read_trout_reg( (UWORD32)rMAC_PHY_REG_RW_DATA); /* Read data from register */
}

/* duplicate write_phy function, but not hold mutex by zhao */
static void internal_change_bank(UWORD8 ra, UWORD32 rd)
{
	UWORD32 val   = 0;
	UWORD32 delay = 0;

	while((host_read_trout_reg((UWORD32)rMAC_PHY_REG_ACCESS_CON) & REGBIT0) == REGBIT0){
		mdelay(1);
		delay++;
		if(delay > 10){
			printk("read phy reg is timeout\n");
			break;
		}
	}
	host_write_trout_reg(rd, (UWORD32)rMAC_PHY_REG_RW_DATA );
	val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
	val &= ~ BIT1;            /* Read/Write bit set to 0 for write */
	val |= BIT0;              /* Access bit set to 1 */

	host_write_trout_reg(val, (UWORD32)rMAC_PHY_REG_ACCESS_CON );
}

void write_dot11_phy_reg(UWORD8 ra, UWORD32 rd)
{
	UWORD32 val   = 0;
	UWORD32 delay = 0;
	UWORD32 v, swich = 0;

	get_phy_mutex();
	read_dot11_phy_reg(0xFF, &v);
	if(v){
		printk("read bank val is %x\n", v);
		/* it the current bank is not bank0, switch to bank0 by zhao*/
		internal_change_bank(0xff,  0x00);
		swich = 1;
	}
	while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON)& REGBIT0) == REGBIT0){
		mdelay(1);
		delay++;
		/* Wait for sometime for the CE-LUT update operation to complete */
		if(delay > 10){
			printk("read phy reg is timeout\n");
			break;
		}
	}

	host_write_trout_reg(rd, (UWORD32)rMAC_PHY_REG_RW_DATA);
	val = ((UWORD8)ra << 2); 
	val &= ~ BIT1;            
	val |= BIT0;              

	host_write_trout_reg(val, (UWORD32)rMAC_PHY_REG_ACCESS_CON);

	if(swich)
		internal_change_bank(0xff,  v);
	put_phy_mutex();
}

