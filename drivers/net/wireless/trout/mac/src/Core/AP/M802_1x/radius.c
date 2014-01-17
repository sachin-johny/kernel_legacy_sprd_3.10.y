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
/*  File Name         : radius.c                                             */
/*                                                                           */
/*  Description       : This file contains all the radius dameon related     */
/*                      functions                                            */
/*                                                                           */
/*  List of Functions : _daemonize                                           */
/*                      allow_signal                                         */
/*                      radiusd                                              */
/*                      update_radius_param                                  */
/*                      start_radius_client                                  */
/*                      update_radius_socket                                 */
/*                      halt_radius_thread                                   */
/*                      recv                                                 */
/*                      send_frame_to_rad_server                             */
/*                      sendto                                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"

#ifdef ENABLE_AP_1X_LINUX_PROCESSING
//#include <linux/config.h>	//shield by chengwg.
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/sysctl.h>
#include <linux/in.h>
#include <linux/utsname.h>
#include <linux/sched.h>
#include <linux/inet.h>
#include <linux/signal.h>
#include <asm/uaccess.h>
#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <linux/random.h>
#include <net/sock.h>
#include <linux/syscalls.h>	//add by chengwg.

#include "radius_client.h"

//#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)	//modify by chengwg.
/*leon liu stripped smp_lock.h*/
//#include <linux/smp_lock.h>
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

/*****************************************************************************/
/* Constants/Macros                                                          */
/*****************************************************************************/

#if !(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
#define _daemonize(s)   daemonize(s)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

/*****************************************************************************/
/* Global Variables Definition                                               */
/*****************************************************************************/

UWORD8 radius_secret[32] = {0};

struct socket      *g_radius_socket  = NULL;
pid_t              g_radius_pid      = -1;
struct sockaddr_in radius_serveraddr = {0};
struct sockaddr_in radius_clientaddr = {0};
struct sockaddr_in new_radius_clientaddr = {0};
BOOL_T g_update_socket               = BTRUE;

/*****************************************************************************/
/* Function Declarations                                                     */
/*****************************************************************************/

static UWORD32 recv(struct socket * sockfd, UWORD8 * data, UWORD32 size,
                    UWORD32 flags);

static WORD32 sendto(struct socket *sockfd, UWORD8 *data, UWORD32 size,
                     UWORD32 flags, struct sockaddr_in *serv_addr,
                     UWORD32 sa_size);

static BOOL_T update_radius_socket(void);

static WORD32 radiusd(void * arg);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

static void   _daemonize(const UWORD8 *procname);
static WORD32 allow_signal(WORD32 sig);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : _daemonize                                               */
/*                                                                           */
/*  Description   : This function implements the deamonize of Linux Kernel   */
/*                                                                           */
/*  Inputs        : 1) Process Name                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function implements the deamonize function of Linux */
/*                  kernel for the given process name                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE on success; BFALSE otherwise                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void _daemonize(const UWORD8 *procname)
{
	/*leon liu stripped lock_kernel*/
	BUG_ON(1);
    //lock_kernel();

    daemonize();
    current->tty = NULL;
    strcpy(current->comm, procname);

    //unlock_kernel();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : allow_signal                                             */
/*                                                                           */
/*  Description   : This function implements the allow_signal of Linux Kernel*/
/*                                                                           */
/*  Inputs        : 1) Process Name                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function implements allow_signal function of Linux  */
/*                  kernel for the signal type                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : 0 on success and Error type for error                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

WORD32 allow_signal(WORD32 sig)
{
    if (sig < 1 || sig > _NSIG)
        return -EINVAL;

#ifdef INIT_SIGHAND

    /* "The test on INIT_SIGHAND is not perfect but will at  */
    /*  least allow this to compile on RedHat kernels."      */
    spin_lock_irq(&current->sighand->siglock);
    sigdelset(&current->blocked, sig);
    /* XXX current->mm? */
    recalc_sigpending();
    spin_unlock_irq(&current->sighand->siglock);

#else /* INIT_SIGHAND */

    spin_lock_irq(&current->sigmask_lock);
    sigdelset(&current->blocked, sig);
    /* XXX current->mm? */
    recalc_sigpending(current);
    spin_unlock_irq(&current->sigmask_lock);

#endif /* INIT_SIGHAND */

    return 0;
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */


/*****************************************************************************/
/*                                                                           */
/*  Function Name : radiusd                                                  */
/*                                                                           */
/*  Description   : This function implements the radius thread               */
/*                                                                           */
/*  Inputs        : 1) Void pointer (Not used)                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function loops and receives all the radius packets  */
/*                  and if valid, passes on to the processing function       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : 0 always                                                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

WORD32 radiusd(void * arg)
{
    UWORD32 len = 0;
    UWORD8  one = 1;
    UWORD8  *data_buff   = NULL;
    UWORD16 rx_buff_size = 1500;

    /* Deamonize and unmask the signkill for this thread */
    _daemonize("IttiamRadiusd");
    allow_signal(SIGKILL);

    /* This buffer will be used to recieve the incoming packets */
    data_buff = kmalloc(rx_buff_size, 0);
    if(data_buff == NULL)
    {
        one = 0;
    }
	
    /* Receive the packet on the port                                  */
    /* If the packet is received, process the packet in the radius FSM */
    while(one)
    {
        len = 0;

        /* Waits for a packet, once a packet has been received, process it */
        while(len <= 0)
        {
              len = recv(g_radius_socket, data_buff, rx_buff_size, 0);
              if(0)//g_update_socket == BTRUE)
              {
                  g_update_socket = BFALSE;
                  if(update_radius_socket() != BTRUE)
                  {
                      one = 0;
                  }
              }
        }
        if(signal_pending(current))
        {
            break;
        }

        /* Process the received packet */
        process_host_rx_radius(data_buff, len);
    }

    /* Free the data buffer */
	if(data_buff != NULL)
	{
    	kfree(data_buff);
    }
    if(g_radius_socket != NULL)
    {
        sock_release(g_radius_socket);
        g_radius_socket = NULL;
    }

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    module_put_and_exit(0);
#else
    return 0;
#endif /* LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_radius_param                                      */
/*                                                                           */
/*  Description   : This function updates Radius Thread parameters on reset  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) radius_serveraddr                                     */
/*                  2) g_local_ip_addr                                       */
/*                  3) g_server_ip_addr                                      */
/*                                                                           */
/*  Processing    : This function updates parameters of Radius Thread on     */
/*                  reset                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE on success; BFALSE otherwise                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_radius_param(void)
{
    UWORD8  ip_addr_str[16] = {0};

    /* Initialize client socket address
    new_radius_clientaddr.sin_family = AF_INET;
    new_radius_clientaddr.sin_port   = htons(g_local_port);
    mem_set(ip_addr_str, 0, 16);
    sprintf(ip_addr_str, "%d.%d.%d.%d", g_local_ip_addr[0],
        g_local_ip_addr[1],g_local_ip_addr[2],g_local_ip_addr[3]);
    radius_clientaddr.sin_addr.s_addr = in_aton(ip_addr_str); */

    /* Initialize server socket address */
    radius_serveraddr.sin_family = AF_INET;
    radius_serveraddr.sin_port   = htons(g_server_port);
    mem_set(ip_addr_str, 0, 16);
    sprintf(ip_addr_str, "%d.%d.%d.%d", g_server_ip_addr[0],
        g_server_ip_addr[1],g_server_ip_addr[2],g_server_ip_addr[3]);
    radius_serveraddr.sin_addr.s_addr = in_aton(ip_addr_str);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_radius_client                                      */
/*                                                                           */
/*  Description   : This function starts the Radius Thread in Linux Kernel   */
/*                  Space                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_radius_socket                                       */
/*                  2) radius_clientaddr                                     */
/*                  3) radius_serveraddr                                     */
/*                  4) g_radius_secret_len                                   */
/*                  5) g_local_ip_addr                                       */
/*                  6) g_local_port                                          */
/*                  7) g_server_ip_addr                                      */
/*                  8) g_server_port                                         */
/*                                                                           */
/*  Processing    : This function starts the Radius Thread in Linux Kernel   */
/*                  space and initializes the thread with given parameters   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE on success; BFALSE otherwise                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T start_radius_client(void)
{
    UWORD32 error;
    UWORD8  ip_addr_str[16] = {0};

    /* Initialize server socket address */
    radius_serveraddr.sin_family = AF_INET;
    radius_serveraddr.sin_port   = htons(g_server_port);
    mem_set(ip_addr_str, 0, 16);
    sprintf(ip_addr_str, "%d.%d.%d.%d", g_server_ip_addr[0],
        g_server_ip_addr[1],g_server_ip_addr[2],g_server_ip_addr[3]);
    radius_serveraddr.sin_addr.s_addr = in_aton(ip_addr_str);

    if(g_radius_pid == -1)
    {
        /* Initialize client socket address */
        radius_clientaddr.sin_family = AF_INET;
        radius_clientaddr.sin_port   = htons(g_local_port);
        radius_clientaddr.sin_addr.s_addr = INADDR_ANY;

        /* Check if the secret key is setup */
        if(g_radius_secret_len == 0)
        {
            return BFALSE;
        }

        g_radius_pid = -1;

        /* Create the socket for the radius client */
        error = sock_create(PF_INET, SOCK_DGRAM, IPPROTO_UDP,
                            &g_radius_socket);
        if (error < 0)
        {
            return BFALSE;
        }

        /* Force atomic allocation since we sometimes send */
        /* messages from interrupt level.                  */
        /* XXX 2.6.0 is just a guess                       */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        g_radius_socket->sk->allocation = GFP_ATOMIC;
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */
        g_radius_socket->sk->sk_allocation = GFP_ATOMIC;
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

        error = (*g_radius_socket->ops->bind)(g_radius_socket,
                (struct sockaddr *)&radius_clientaddr,
                sizeof(radius_clientaddr));

        if (error < 0)
        {
            if(g_radius_socket != NULL)
            {
                sock_release(g_radius_socket);
                g_radius_socket = NULL;
            }
            return BFALSE;
        }

        /* Start the kernet thread */
        g_radius_pid = kernel_thread(radiusd, 0,
                                     CLONE_FS|CLONE_FILES|CLONE_SIGHAND);
        if (g_radius_pid < 0)
        {
            if(g_radius_socket != NULL)
            {
                sock_release(g_radius_socket);
                g_radius_socket = NULL;
            }
            return BFALSE;
        }
    }
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_radius_socket                                     */
/*                                                                           */
/*  Description   : This function updates the radius socket in the linux     */
/*                  kernel space                                             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_radius_socket                                       */
/*                  2) radius_clientaddr                                     */
/*                  3) new_radius_clientaddr                                 */
/*                                                                           */
/*  Processing    : This function updates Radius Thread in Linux Kernel      */
/*                  space                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T update_radius_socket(void)
{
    if(g_radius_pid != -1)
    {
        UWORD32 error;

        radius_clientaddr = new_radius_clientaddr;
        if(g_radius_socket != NULL)
        {
             sock_release(g_radius_socket);
             g_radius_socket = NULL;
        }

        /* Create the socket for the radius client */
        error = sock_create(PF_INET, SOCK_DGRAM, IPPROTO_UDP, &g_radius_socket);
        if (error < 0)
        {
            return BFALSE;
        }

        /* Force atomic allocation since we sometimes send */
        /* messages from interrupt level.                  */
        /* XXX 2.6.0 is just a guess                       */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        g_radius_socket->sk->allocation = GFP_ATOMIC;
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */
        g_radius_socket->sk->sk_allocation = GFP_ATOMIC;
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

        error = (*g_radius_socket->ops->bind)(g_radius_socket,
                (struct sockaddr *)&radius_clientaddr, sizeof(radius_clientaddr));
        if (error < 0)
        {
            if(g_radius_socket != NULL)
            {
                sock_release(g_radius_socket);
                g_radius_socket = NULL;
            }
            return BFALSE;
        }
    }
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : halt_radius_thread                                       */
/*                                                                           */
/*  Description   : This function stops the Radius Thread in Linux Kernel    */
/*                  Space                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : 1) g_radius_socket                                       */
/*                                                                           */
/*  Processing    : This function stops the Radius Thread in Linux Kernel    */
/*                  space and frees the threads parameters                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void halt_radius_thread(void)
{
		/*leon liu stripped lock_kernel*/
		BUG_ON(1);
    if(g_radius_pid != -1)
    {
        //lock_kernel();
        //kill_proc(g_radius_pid, SIGKILL, 1);	//need modify!!!
        //sys_kill(SIGKILL, g_radius_pid);
        //unlock_kernel();
        g_radius_pid = -1;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : recv                                                     */
/*                                                                           */
/*  Description   : This function implements packet receive on a socket      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the socket                                 */
/*                  2) Pointer to the receive buffer                         */
/*                  3) Size of the receive buffer                            */
/*                  4) Receive function flags                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function implements packet receive on a socket      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Size of the received packet                              */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 recv(struct socket * sockfd, UWORD8 * data, UWORD32 size,
             UWORD32 flags)
{

    struct msghdr msg;
    struct iovec iov;
    mm_segment_t oldfs;
    struct sockaddr_in  sin;
    UWORD32 len = 0;

    msg.msg_name = &sin;
    msg.msg_namelen = sizeof(struct sockaddr_in);
    iov.iov_base = data;
    iov.iov_len = size;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = NULL;
    msg.msg_controllen = 0;
    msg.msg_flags = flags;

    oldfs = get_fs();
    set_fs(KERNEL_DS);
    len = sock_recvmsg(sockfd, &msg, iov.iov_len, 0);

    if(memcmp(&radius_clientaddr, &new_radius_clientaddr,
              sizeof(struct sockaddr_in)) != 0)
    {
        g_update_socket = BTRUE;
    }

    /* If the size is more than zero, check for the source address */
    /* It should be same as the server address                     */
    if(len > 0)
    {
         if(radius_serveraddr.sin_addr.s_addr != sin.sin_addr.s_addr)
         {
             len = 0;
         }
    }
    set_fs(oldfs);
    return len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_frame_to_rad_server                                 */
/*                                                                           */
/*  Description   : This function implements packet send function            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit data                          */
/*                  2) Size of the transmit data                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sends the given packet using sendto call   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE on success; BFALSE otherwise                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_frame_to_rad_server(UWORD8 *data, UWORD32 len)
{
    if((g_radius_socket != NULL) && (data != NULL) && (len != 0))
    {
        /* Send the packet if the socket is valid */
        UWORD32 temp = sendto(g_radius_socket, data, len, 0,
               (struct sockaddr_in *)&radius_serveraddr,
               sizeof(struct sockaddr_in));

       /* If all the bytes have been transmitted, return TRUE */
       if(temp == len)
       {
            return BTRUE;
       }
    }
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sendto                                                   */
/*                                                                           */
/*  Description   : This function implements packet send on a socket         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the socket                                 */
/*                  2) Pointer to the transmit buffer                        */
/*                  3) Size of the transmit data                             */
/*                  4) Transmit flags                                        */
/*                  5) Pointer to the destination socket address             */
/*                  6) Size of the destination socket address                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function implements packet send on a socket         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Number of bytes transmitted                              */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

WORD32 sendto(struct socket *sockfd, UWORD8 *data, UWORD32 size, UWORD32 flags,
              struct sockaddr_in *serv_addr, UWORD32 sa_size)
{

    struct msghdr msg;
    struct iovec iov;
    mm_segment_t oldfs;
    WORD32 i;

    msg.msg_name = serv_addr;
    msg.msg_namelen = sa_size;
    iov.iov_base = data;
    iov.iov_len = size;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = NULL;
    msg.msg_controllen = 0;
    msg.msg_flags = MSG_NOSIGNAL;

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    i = sock_sendmsg(sockfd, &msg, iov.iov_len);

    set_fs(oldfs);

    return i;
}
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */

#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */

