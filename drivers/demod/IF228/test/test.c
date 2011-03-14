#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  
#include <string.h>
#include <getopt.h>
#include <signal.h> 
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>

#include <fcntl.h>
#include <sys/mman.h>
#include "../include/inno_io.h"

#define INNO_CTL        "/dev/innoctl"

#define BUF_LEN         0x11000

static int uam_fd = -1;

static void dump_mem(unsigned char *buf, int len)
{
        int i;
        int a = (len / 8)*8;
        int b = len % 8;
        for (i = 0; i < a; i+=8) {
                printf("%02x %02x %02x %02x %02x %02x %02x %02x\n",
                        buf[i], buf[i+1], buf[i+2], buf[i+3],
                        buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
        }
        for (i = 0; i < b; i++) {
                printf("%02x ", buf[a+i]);
        }
        printf("\n");
}

int INNO_UAM_Transfers(unsigned char *pBufIn, unsigned int bufInLen, unsigned char *pBufOut, unsigned int *pBufOutLen, unsigned short *sw)
{        
        int ret = 0;      
        struct uam_param param; 
        int i;  
        
        param.buf_in = pBufIn;
        param.len_in = bufInLen;
        param.buf_out = pBufOut;
        param.len_out = pBufOutLen;
        
        ret = ioctl(uam_fd, INNO_IO_UAM_TRANSFER, &param);     
        if (ret < 0) {                             
                return -1;      
        }       
        printf("%s:len_out = %d\n", __func__, *pBufOutLen);      
 
        *sw = param.sw;
        return ret;
}

unsigned int Get16Bit(unsigned char *pBuff)
{    
        return (pBuff[0] << 8) | pBuff[1];
}

int Card_SelectFile_2G(unsigned char *path, unsigned char pathLen, unsigned int *fileSize, unsigned int *recordLen) 
{        
        unsigned char cmd[8] = "\xa0\xa4\x00\x00\x02\xFF\xFF";  
        unsigned char lpRes[256];       
        unsigned int nResLen = 0;       
        unsigned int nStatus = 0;       
        unsigned char i;        

        int ret = 0;   

        printf("Card_SelectFile_2G\n"); 
        if (pathLen % 2 || pathLen == 0) {               
                printf("Card_SelectFile_2G err\n");             
                goto err;       
        }       

        for (i = 0; i < pathLen; i += 2) {               
                cmd[5] = path[i];               
                cmd[6] = path[i + 1];           
                nStatus = 0;            
                        
                nResLen = 256;
                ret = INNO_UAM_Transfers(cmd, 7, lpRes, &nResLen, (unsigned short *)&nStatus);             
                printf("Card_SelectFile ret = 0x%x\r\n", ret);          
                if (ret < 0) 
                        goto err;       

                if ((nStatus != 0x9000) && ((nStatus & 0x9F00) != 0x9F00))                      
                        goto err;       
        }       

        printf("Card_SelectFile nStatus = 0x%x\r\n", nStatus);  
        memcpy(cmd, "\xa0\xc0\x00\x00\x0f", 5); 

        cmd[4] = (unsigned char)nStatus & 0xff; 
        /* get rsp */   
        nResLen = 256;
        ret = INNO_UAM_Transfers(cmd, 5, lpRes, &nResLen, (unsigned short *)&nStatus);     
        printf("Card_SelectFile getrsp nReslen= %d, ret = 0x%x\r\n", nResLen, ret);    

        if (ret < 0) 
                goto err;       

        if (nResLen != cmd[4])          
                goto err;       

        if (nStatus == 0x9000)  {               
                *fileSize = Get16Bit(lpRes + 2);                
                *recordLen = (unsigned int)lpRes[14];   
        } else {               
                *fileSize = 0;          
                *recordLen = 0;         
                goto err;       
        }       

        
        printf("Card_SelectFile:nStatus = 0x%x, filesize = %d, recordlen = %d\n", nStatus, *fileSize, *recordLen);  
        goto out; 
err:
        ret = -1;
out:
        return ret;
}

int Card_ReadBin_2G(unsigned int offset, unsigned char dataLen, unsigned char *lpRes, unsigned int *nResLen) 
{       
        unsigned char cmd[8] = "\xA0\xB0\x00\x00\x00";  
        unsigned int nStatus = 0;       
        int ret = 0;   

        printf("Card_ReadBin_2G\n");    

        cmd[2] = (unsigned char)(offset >> 8);  
        cmd[3] = (unsigned char)(offset);       
        cmd[4] = (unsigned char)(dataLen);      

        ret = INNO_UAM_Transfers(cmd, 5, lpRes, nResLen, (unsigned short *)&nStatus);   

        /*datalen1*/       
        if (ret < 0)               
                goto err;       

        if (dataLen != *nResLen)                
                goto err;       
        if (0x9000 != nStatus)          
                goto err;       

        return 0;
err:
        ret = -1;
        return ret;
}

int MBBMS_TEL_GetSDVersion_2G(void) 
{
        unsigned char res[256]; 
        unsigned int resLen = 0;        
        unsigned int fileSize = 0;      
        unsigned int recordLen = 0;     
        unsigned int i; 

        int ret = 0;

        printf("MBBMS_TEL_GetSDVersion_2G\n");  
        unsigned char cmd[7] = "\x3F\x00\x9F\x01";              

        ret = Card_SelectFile_2G(cmd, 4, &fileSize, &recordLen);
        if (ret < 0) {                               
                printf("Card_SelectFile_2G fail  \n");          
                goto err;
        }       

        if (fileSize != 4) {                               
                goto err;
        }       

        resLen = 256;
        ret = Card_ReadBin_2G(0, (unsigned char)fileSize, res, &resLen);
        if (ret < 0) {                               
                printf("Card_ReadBin_2G fail  \n");             
                goto err;
        }       

        printf("%s: res:\n", __func__);
        dump_mem(res, resLen); 
        return ret;  
err:
        ret = -1;
        return 0;
}


int lgx_test(int fd[], int num)
{
        unsigned char *mem[4];
        int i;
        unsigned char *buf;
        fd_set read_set;
        struct timeval read_timeout = {0, 0};
        int ret;
        
        buf = (unsigned char*)malloc(BUF_LEN);
        memset(buf, 0, BUF_LEN);
        for (i = 0; i < num; i++) 
                mem[i] = mmap(NULL, 0x10000, PROT_READ, MAP_SHARED, fd[i], 0);

        FD_ZERO(&read_set);
        for (i = 0; i < num; i++) {
                FD_SET(fd[i], &read_set);
        }
        read_timeout.tv_sec = 3;        //wait 3 seconds to poll
        ret = select(fd[num-1] + 1, &read_set, NULL, NULL, &read_timeout);
       //         ret = select(fd_lg0+1, &read_set, NULL, NULL, NULL);
        if(ret < 0){
                  printf("select error!\n");
        }else if(ret == 0){
                  printf("select time out!\n");
        }else{
                for (i = 0; i < num; i++) {
                        if(FD_ISSET(fd[i], &read_set)){
                                printf("lg %d select return!\n",i);
                         /* 
                                ret = read(fd[i], buf, BUF_LEN);
                                if(ret > 0){
                                        printf("lg %d data available(0x%x)\n", i, ret);
                                        dump_mem(buf, 20);
                                }
                         */
                               dump_mem(mem[i], 20);
                        }
                }
        }
        for (i = 0; i < num; i++) 
                munmap(mem[i], 0x10000);
        return 0;
}

int main(int argc, char** argv)
{
        int fd = -1;
        unsigned char ts_num, ts_start, ts_count, ts_mod;
        int i, j, tmp;
        char devname[30];
        char cmd;
        int exitf = 0;
 
        printf("-c control test\n");
        printf("-l logic channel test\n");
        printf("-u uam test\n");
        printf("select:");

        cmd = getchar();
        getchar();
        if (cmd == 'c') {
                fd = open(INNO_CTL, O_RDWR);
                if (fd < 0) {
                        printf("ctl open failed\n");
                        return 0;
                }

                while(exitf == 0) {
menu_ctl:
                        printf("\n");
                        printf("-f set/get freq\n");
                        printf("-t set/get time slot config\n");
                        printf("-s get system status\n");
                        printf("-v get fw version\n");
                        printf("-e get error info\n");
                        printf("-l goto lgx menu");
                        printf("-x exit\n");
                        printf("select:");
                        cmd = getchar();
                        getchar();
                        switch (cmd) {
                        case 's':
                        {
                               struct sys_status status; 
                                ioctl(fd, INNO_IO_GET_SYS_STATUS, &status);
				//modified by mahanghong 20110118
                                printf("sync = %d, strength = %d, cur freq = %d, ldpc = %d, rs = %d,errstatus = 0x%x\n",
                                        status.sync,
                                        status.signal_strength,
                                        status.cur_freq,
                                        status.ldpc_err_percent,
                                        status.rs_err_percent,
   					//add by mahanghong 20110118
                                        status.err_status); 
                                break;
                        }
                        case 'v':
                        {
                                unsigned int version; 
                                ioctl(fd, INNO_IO_GET_FW_VERSION, &version);
                                printf("version = %x\n", version);
                                break;
                        }
                        case 'e':
                        {
                                struct err_info info;
                                ioctl(fd, INNO_IO_GET_ERR_INFO, &info);
                                printf("ldpc total= %d, ldpc error = %d, rs total = %d, rs error = %d, BER = %d, SNR = %d\n",
                                        info.ldpc_total_count,
                                        info.ldpc_error_count,
                                        info.rs_total_count,
                                        info.rs_error_count,
                                        info.BER,
                                        info.SNR); 
                                break;
                        }
                        case 'f':
                        {       
                                unsigned char freq_dot;
                                printf("-s  set freq\n");
                                printf("-g  get freq\n");
                                printf("select:");
                                cmd = getchar();
                                getchar();
                                if (cmd == 's') {
                                        printf("freq dot:");
                                        scanf("%d", &freq_dot);
                                        getchar();
                                        ioctl(fd, INNO_IO_SET_FREQUENCY , &freq_dot); 
                                } else if (cmd == 'g') {
                                        ioctl(fd, INNO_IO_GET_FREQUENCY , &freq_dot); 
                                        printf("freq dot = %d\n", freq_dot); 
                                }
                                break;
                        }
                        case 't':
                        {
                                struct ch_config config;
                                printf("-s  set channel config\n");
                                printf("-g  get channel config\n");
                                printf("select:");
                                cmd = getchar();
                                getchar();
                                if (cmd == 's') {
//                                        printf("config:");
//                                        scanf("%d %d %d %x", 
//                                                &config.ch_id,
//                                                &config.start_timeslot,
//                                                &config.timeslot_count,
//                                                &config.demod_config);
//                                        getchar();

                                        config.ch_id = 1;
                                        config.start_timeslot = 19;
                                        config.timeslot_count = 4;
                                        config.demod_config = 0x6c;
                                        ioctl(fd, INNO_IO_SET_CH_CONFIG , &config); 
                                } else if (cmd == 'g') {
                                        printf("channel id:");
                                        scanf("%d", &config.ch_id);
                                        getchar();
                                        ioctl(fd, INNO_IO_GET_CH_CONFIG , &config); 
                                        
                                        printf("ch%d: ts =  %d, count =  %d config = %x", 
                                                config.ch_id,
                                                config.start_timeslot,
                                                config.timeslot_count,
                                                config.demod_config);
                                                 
                                }
                                break;
                        }
                        case 'l':
                            goto menu_lgx;
                            break;
                        case 'x':
                                exitf = 1;
                                break;
                        default:
                                break;
                        }
                }                        

                close(fd);

        } else if (cmd == 'l') {
                printf("channel:"); 
                scanf("%d", &tmp);
                getchar();
                sprintf(devname, "/dev/innolg%d", tmp);
                fd = open(devname, O_RDWR);
                if (fd < 0)
                {
                    printf("open logic channel device fail!\n");
                    return 0;
                }
                while(1)
                {
menu_lgx:
                    printf("-s start test\n");
                    printf("-c goto control menu\n");
                    printf("-b run in background.\n");
                    printf("select:");
                    cmd = getchar();
                    getchar();
                    if (cmd == 's')
                    {
                        lgx_test(&fd, 1);
                    }
                    else if (cmd == 'c')
                    {
                        goto menu_ctl;
//                        close(fd);
                    }
                    else if (cmd == 'x')
                    {
                        unsigned int pid = fork();
                        if (pid > 0)
                        {
                            exit(0);
                            printf("Run in background. pid = %d\n", pid);
                            
                        }
                        else
                        {
                             printf(" Can't run in background!\n");
                             exit(-1);
                        }
                    }
                    else
                    {
                        exit(-1);
                    }
                }
        } else if (cmd == 'u') {
               
                uam_fd = open("/dev/innouam0", O_RDWR);
                if (uam_fd < 0) {
                        printf("/dev/innouam0 open failed\n");
                        return 0;
                }
                MBBMS_TEL_GetSDVersion_2G();
                close(uam_fd);
        }
        return 0;
} 
