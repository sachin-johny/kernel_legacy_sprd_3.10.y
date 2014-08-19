//------------------------- Data Prototype ------------------------------------

#define CHIP_ENDIAN_LITTLE

#define REG_CONF1     ( MAX2769_REG_BASE + 0x0*4 )
#define REG_CONF2     ( MAX2769_REG_BASE + 0x1*4 )
#define REG_CONF3     ( MAX2769_REG_BASE + 0x2*4 )
#define REG_PLLCONF   ( MAX2769_REG_BASE + 0x3*4 )
#define REG_DIV       ( MAX2769_REG_BASE + 0x4*4 )
#define REG_FDIV      ( MAX2769_REG_BASE + 0x5*4 )
#define REG_STRM      ( MAX2769_REG_BASE + 0x6*4 )
#define REG_CLK       ( MAX2769_REG_BASE + 0x7*4 )


typedef union _conf1_tag
{ 


#ifdef CHIP_ENDIAN_LITTLE

    struct _conf1_map
    {
        volatile unsigned int fgain              :  1 ; // [      0] 
        volatile unsigned int fcenx              :  1 ; // [      1] 
        volatile unsigned int f3or5              :  1 ; // [      2] 
        volatile unsigned int fbw                :  2 ; // [ 4 :  3] 
        volatile unsigned int fcen               :  6 ; // [10 :  5] 
        volatile unsigned int anten              :  1 ; // [     11] 
        volatile unsigned int mixen              :  1 ; // [     12] 
        volatile unsigned int lnamode            :  2 ; // [14 : 13] 
        volatile unsigned int mixpole            :  1 ; // [     15] 
        volatile unsigned int imix               :  2 ; // [17 : 16] 
        volatile unsigned int ilo                :  2 ; // [19 : 18] 
        volatile unsigned int ilna2              :  2 ; // [21 : 20] 
        volatile unsigned int ilna1              :  4 ; // [25 : 22] 
        volatile unsigned int idle               :  1 ; // [     26] 
        volatile unsigned int chipen             :  1 ; // [     27] 
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _conf1_map
    {
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
        volatile unsigned int chipen             :  1 ; // [     27] 
        volatile unsigned int idle               :  1 ; // [     26] 
        volatile unsigned int ilna1              :  4 ; // [25 : 22] 
        volatile unsigned int ilna2              :  2 ; // [21 : 20] 
        volatile unsigned int ilo                :  2 ; // [19 : 18] 
        volatile unsigned int imix               :  2 ; // [17 : 16] 
        volatile unsigned int mixpole            :  1 ; // [     15] 
        volatile unsigned int lnamode            :  2 ; // [14 : 13] 
        volatile unsigned int mixen              :  1 ; // [     12] 
        volatile unsigned int anten              :  1 ; // [     11] 
        volatile unsigned int fcen               :  6 ; // [10 :  5] 
        volatile unsigned int fbw                :  2 ; // [ 4 :  3] 
        volatile unsigned int f3or5              :  1 ; // [      2] 
        volatile unsigned int fcenx              :  1 ; // [      1] 
        volatile unsigned int fgain              :  1 ; // [      0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} CONF1_U ;


//------------------------- Data Prototype ------------------------------------
typedef union _conf2_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _conf2_map
    {
        volatile unsigned int dieid              :  2 ; // [ 1 :  0] 
        volatile unsigned int reserved_2         :  1 ; // [      2] 
        volatile unsigned int loen               :  1 ; // [      3] 
        volatile unsigned int drvcfg             :  2 ; // [ 5 :  4] 
        volatile unsigned int bits               :  3 ; // [ 8 :  6] 
        volatile unsigned int format             :  2 ; // [10 :  9] 
        volatile unsigned int agcmode            :  2 ; // [12 : 11] 
        volatile unsigned int reserved_13        :  2 ; // [14 : 13] 
        volatile unsigned int gainref            : 12 ; // [26 : 15] 
        volatile unsigned int iqen               :  1 ; // [     27] 
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _conf2_map
    {
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
        volatile unsigned int iqen               :  1 ; // [     27] 
        volatile unsigned int gainref            : 12 ; // [26 : 15] 
        volatile unsigned int reserved_13        :  2 ; // [14 : 13] 
        volatile unsigned int agcmode            :  2 ; // [12 : 11] 
        volatile unsigned int format             :  2 ; // [10 :  9] 
        volatile unsigned int bits               :  3 ; // [ 8 :  6] 
        volatile unsigned int drvcfg             :  2 ; // [ 5 :  4] 
        volatile unsigned int loen               :  1 ; // [      3] 
        volatile unsigned int reserved_2         :  1 ; // [      2] 
        volatile unsigned int dieid              :  2 ; // [ 1 :  0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} CONF2_U ;


//------------------------- Data Prototype ------------------------------------
typedef union _conf3_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _conf3_map
    {
        volatile unsigned int strmrst               :  1 ; // [      0] 
        volatile unsigned int datsyncen             :  1 ; // [      1] 
        volatile unsigned int timesyncen            :  1 ; // [      2] 
        volatile unsigned int stampen               :  1 ; // [      3] 
        volatile unsigned int strmbits              :  2 ; // [ 5 :  4] 
        volatile unsigned int strmcount             :  3 ; // [ 8 :  6] 
        volatile unsigned int strmstop              :  1 ; // [      9] 
        volatile unsigned int strmstart             :  1 ; // [     10] 
        volatile unsigned int strmen                :  1 ; // [     11] 
        volatile unsigned int pgaqen                :  1 ; // [     12] 
        volatile unsigned int pgaien                :  1 ; // [     13] 
        volatile unsigned int reserved_14           :  1 ; // [     14] 
        volatile unsigned int fhipen                :  1 ; // [     15] 
        volatile unsigned int filten                :  1 ; // [     16] 
        volatile unsigned int fofsten               :  1 ; // [     17] 
        volatile unsigned int drven                 :  1 ; // [     18] 
        volatile unsigned int adcen                 :  1 ; // [     19] 
        volatile unsigned int hiloaden              :  1 ; // [     20] 
        volatile unsigned int fslowen               :  1 ; // [     21] 
        volatile unsigned int gainin                :  6 ; // [27 : 22] 
        volatile unsigned int reserved_28           :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _conf3_map
    {
        volatile unsigned int reserved_28           :  4 ; // [31 : 28] 
        volatile unsigned int gainin                :  6 ; // [27 : 22] 
        volatile unsigned int fslowen               :  1 ; // [     21] 
        volatile unsigned int hiloaden              :  1 ; // [     20] 
        volatile unsigned int adcen                 :  1 ; // [     19] 
        volatile unsigned int drven                 :  1 ; // [     18] 
        volatile unsigned int fofsten               :  1 ; // [     17] 
        volatile unsigned int filten                :  1 ; // [     16] 
        volatile unsigned int fhipen                :  1 ; // [     15] 
        volatile unsigned int reserved_14           :  1 ; // [     14] 
        volatile unsigned int pgaien                :  1 ; // [     13] 
        volatile unsigned int pgaqen                :  1 ; // [     12] 
        volatile unsigned int strmen                :  1 ; // [     11] 
        volatile unsigned int strmstart             :  1 ; // [     10] 
        volatile unsigned int strmstop              :  1 ; // [      9] 
        volatile unsigned int strmcount             :  3 ; // [ 8 :  6] 
        volatile unsigned int strmbits              :  2 ; // [ 5 :  4] 
        volatile unsigned int stampen               :  1 ; // [      3] 
        volatile unsigned int timesyncen            :  1 ; // [      2] 
        volatile unsigned int datsyncen             :  1 ; // [      1] 
        volatile unsigned int strmrst               :  1 ; // [      0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} CONF3_U ;

//------------------------- Data Prototype ------------------------------------



typedef union _pllconf_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _pllconf_map
    {
        volatile unsigned int reserved_0          :  2 ; // [ 1 :  0] 
        volatile unsigned int pwrsav              :  1 ; // [      2] 
        volatile unsigned int int_pll             :  1 ; // [      3] 
        volatile unsigned int cptest              :  3 ; // [ 6 :  4] 
        volatile unsigned int reserved_7          :  1 ; // [      7] 
        volatile unsigned int pfden               :  1 ; // [      8] 
        volatile unsigned int icp                 :  1 ; // [      9] 
        volatile unsigned int ldmux               :  4 ; // [13 : 10] 
        volatile unsigned int xtalcap             :  5 ; // [18 : 14] 
        volatile unsigned int ixtal               :  2 ; // [20 : 19] 
        volatile unsigned int refdiv              :  2 ; // [22 : 21] 
        volatile unsigned int reserved_23         :  1 ; // [     23] 
        volatile unsigned int refouten            :  1 ; // [     24] 
        volatile unsigned int reserved_25         :  1 ; // [     25] 
        volatile unsigned int ivco                :  1 ; // [     26] 
        volatile unsigned int vcoen               :  1 ; // [     27] 
        volatile unsigned int reserved_28         :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _pllconf_map
    {
        volatile unsigned int reserved_28         :  4 ; // [31 : 28] 
        volatile unsigned int vcoen               :  1 ; // [     27] 
        volatile unsigned int ivco                :  1 ; // [     26] 
        volatile unsigned int reserved_25         :  1 ; // [     25] 
        volatile unsigned int refouten            :  1 ; // [     24] 
        volatile unsigned int reserved_23         :  1 ; // [     23] 
        volatile unsigned int refdiv              :  2 ; // [22 : 21] 
        volatile unsigned int ixtal               :  2 ; // [20 : 19] 
        volatile unsigned int xtalcap             :  5 ; // [18 : 14] 
        volatile unsigned int ldmux               :  4 ; // [13 : 10] 
        volatile unsigned int icp                 :  1 ; // [      9] 
        volatile unsigned int pfden               :  1 ; // [      8] 
        volatile unsigned int reserved_7          :  1 ; // [      7] 
        volatile unsigned int cptest              :  3 ; // [ 6 :  4] 
        volatile unsigned int int_pll             :  1 ; // [      3] 
        volatile unsigned int pwrsav              :  1 ; // [      2] 
        volatile unsigned int reserved_0          :  2 ; // [ 1 :  0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} PLLCONF_U ;






//------------------------- Data Prototype ------------------------------------

typedef union _strm_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _strm_map
    {
        volatile unsigned int framecount            : 28 ; // [27 :  0] 
        volatile unsigned int reserved_28           :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _strm_map
    {
        volatile unsigned int reserved_28           :  4 ; // [31 : 28] 
        volatile unsigned int framecount            : 28 ; // [27 :  0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} STRM_U ;

//------------------------- Data Prototype ------------------------------------

typedef union _fdiv_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _fdiv_map
    {
        volatile unsigned int reserved_0         :  8 ; // [ 7 :  0] 
        volatile unsigned int fdiv               : 20 ; // [27 :  8] 
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _fdiv_map
    {
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
        volatile unsigned int fdiv               : 20 ; // [27 :  8] 
        volatile unsigned int reserved_0         :  8 ; // [ 7 :  0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} FDIV_U ;






//------------------------- Data Prototype ------------------------------------


typedef union _clk_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _clk_map
    {
        volatile unsigned int mode               :  1 ; // [      0] 
        volatile unsigned int serclk             :  1 ; // [      1] 
        volatile unsigned int adcclk             :  1 ; // [      2] 
        volatile unsigned int fclkin             :  1 ; // [      3] 
        volatile unsigned int m_cnt              : 12 ; // [15 :  4] 
        volatile unsigned int l_cnt              : 12 ; // [27 : 16] 
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _clk_map
    {
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
        volatile unsigned int l_cnt              : 12 ; // [27 : 16] 
        volatile unsigned int m_cnt              : 12 ; // [15 :  4] 
        volatile unsigned int fclkin             :  1 ; // [      3] 
        volatile unsigned int adcclk             :  1 ; // [      2] 
        volatile unsigned int serclk             :  1 ; // [      1] 
        volatile unsigned int mode               :  1 ; // [      0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} CLK_U ;



//------------------------- Data Prototype ------------------------------------
typedef union _div_tag
{ 

#ifdef CHIP_ENDIAN_LITTLE

    struct _div_map
    {
        volatile unsigned int reserved_0         :  3 ; // [ 2 :  0] 
        volatile unsigned int rdiv               : 10 ; // [12 :  3] 
        volatile unsigned int ndiv               : 15 ; // [27 : 13] 
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
    } mBits ;

#else

    struct _div_map
    {
        volatile unsigned int reserved_28        :  4 ; // [31 : 28] 
        volatile unsigned int ndiv               : 15 ; // [27 : 13] 
        volatile unsigned int rdiv               : 10 ; // [12 :  3] 
        volatile unsigned int reserved_0         :  3 ; // [ 2 :  0] 
    } mBits ;

#endif // CHIP_ENDIAN_LITTLE

    volatile unsigned int dwValue ;
} DIV_U ;

/************************************************************************************************/
 CONF1_U                   reg_CONF1,*p_CONF1;
 CONF2_U                   reg_CONF2,*p_CONF2;
 CONF3_U                   reg_CONF3,*p_CONF3;
CLK_U                         reg_CLK,*p_CLK;
 PLLCONF_U               reg_PLLCONF,*p_PLLCONF;
FDIV_U                       reg_DIV,*p_FDIV;
STRM_U                    reg_STRM,*p_STRM;
DIV_U                       reg_FDIV,*p_DIV;





void max_data_init()
{
p_CONF1  = &reg_CONF1;
p_CONF2  = &reg_CONF2;
p_CONF3= &reg_CONF3;
p_CLK= &reg_CLK;
p_PLLCONF= &reg_PLLCONF;
p_DIV= &reg_DIV;
p_STRM= &reg_STRM;
p_FDIV= &reg_FDIV;

    reg_CONF1.dwValue= 0; 
    reg_CONF2.dwValue= 0; 
    reg_CONF3.dwValue= 0; 
    reg_PLLCONF.dwValue= 0; 
    reg_DIV.dwValue= 0; 
    reg_FDIV.dwValue= 0; 
    reg_STRM.dwValue= 0; 
    reg_CLK.dwValue= 0; 


    //----------------------------------------------------------
    //IF gain gain+6dB
    p_CONF1->mBits.fgain      = 1   ; // [      0]
    //polyphase bandpass filter
    p_CONF1->mBits.fcenx      = 1   ; // [      1]
    //5-order bandpass IF filter
    p_CONF1->mBits.f3or5      = 0   ; // [      2]
    //IF filter bandwidth 2.5MHz
    p_CONF1->mBits.fbw        = 0   ; // [ 4 :  3]
    //IF center freqency 
    p_CONF1->mBits.fcen       = 0xd ; // [10 :  5]
    //antenna bias voltage setting
    p_CONF1->mBits.anten      = 0   ; // [     11]
    p_CONF1->mBits.mixen      = 1   ; // [     12]
    //p_CONF1->mBits.lnamode    = 2   ; // [14 : 13]
	p_CONF1->mBits.lnamode    = 1   ; // [14 : 13]
	p_CONF1->mBits.mixpole    = 0   ; // [     15]
    p_CONF1->mBits.imix       = 1   ; // [17 : 16]
    p_CONF1->mBits.ilo        = 2   ; // [19 : 18]
    p_CONF1->mBits.ilna2      = 2   ; // [21 : 20]
    p_CONF1->mBits.ilna1      = 8   ; // [25 : 22]
    p_CONF1->mBits.idle       = 0   ; // [     26]
    p_CONF1->mBits.chipen     = 1   ; // [     27]

    printk("config1:%x\n",p_CONF1->dwValue);

    //----------------------------------------------------------
    p_CONF2->mBits.dieid      = 0   ; // [ 1 :  0]
    p_CONF2->mBits.loen       = 1   ; // [      3]
    //0:cmos 1:differential  2/3:analog
    //p_CONF2->mBits.drvcfg     = 0   ; // [ 5 :  4]
    p_CONF2->mBits.drvcfg     = 0   ; // [ 5 :  4]
    //0: 1bit 1:1.5bit 2:2bit 3:2.5bit 4:3bit
    p_CONF2->mBits.bits       = 2   ; // [ 8 :  6]
    //0: unsigned  1:sign and magnitude 2/3: two's
    //complement binary
    p_CONF2->mBits.format     = 2   ; // [10 :  9]
    //0:I/Q independent 1:locked each other
    //2:set by SPI
    //p_CONF2->mBits.agcmode    = 0   ; // [12 : 11]
    p_CONF2->mBits.agcmode    = 2   ; // [12 : 11]
    //PDM 30% = 170/512
    p_CONF2->mBits.gainref    = 300 ; // [26 : 15]
    //I/Q channel enable
    p_CONF2->mBits.iqen       = 1   ; // [     27]
    printk("config2:%x\n",p_CONF2->dwValue);

    //----------------------------------------------------------
    p_CONF3->mBits.strmrst     = 0    ; // [      0]
    p_CONF3->mBits.datsyncen   = 0    ; // [      1]
    p_CONF3->mBits.timesyncen  = 1    ; // [      2]
    p_CONF3->mBits.stampen     = 1    ; // [      3]
    p_CONF3->mBits.strmbits    = 1    ; // [ 5 :  4]
    p_CONF3->mBits.strmcount   = 7    ; // [ 8 :  6]
    p_CONF3->mBits.strmstop    = 0    ; // [      9]
    p_CONF3->mBits.strmstart   = 0    ; // [     10]
    p_CONF3->mBits.strmen      = 0    ; // [     11]
    //Q channel PGA enable
    p_CONF3->mBits.pgaqen      = 1    ; // [     12]
    //I channel PGA enable
    p_CONF3->mBits.pgaien      = 1    ; // [     13]
    p_CONF3->mBits.reserved_14 = 1    ; // [     14] 
    //highpass couping enable
    p_CONF3->mBits.fhipen      = 1    ; // [     15]
    //IF filter enable
    p_CONF3->mBits.filten      = 1    ; // [     16]
    //DC offset cancellation
    p_CONF3->mBits.fofsten     = 1    ; // [     17]
    p_CONF3->mBits.drven       = 1    ; // [     18]
    p_CONF3->mBits.adcen       = 1    ; // [     19]
    p_CONF3->mBits.hiloaden    = 0    ; // [     20]
    p_CONF3->mBits.fslowen     = 1    ; // [     21]
    //Gain settings
    p_CONF3->mBits.gainin      = 63    ;//0x3A ; // [27 : 22]
    printk("config3:%x\n",p_CONF3->dwValue);
    
    //----------------------------------------------------------
    p_PLLCONF->mBits.pwrsav      = 0    ; // [      2]
    p_PLLCONF->mBits.int_pll     = 1    ; // [      3]
    p_PLLCONF->mBits.cptest      = 0    ; // [ 6 :  4]
    p_PLLCONF->mBits.pfden       = 0    ; // [      8]
    p_PLLCONF->mBits.icp         = 0    ; // [      9]
    p_PLLCONF->mBits.ldmux       = 0    ; // [13 : 10]
    p_PLLCONF->mBits.xtalcap     = 0x10 ; // [18 : 14]
    p_PLLCONF->mBits.ixtal       = 1    ; // [20 : 19]
    p_PLLCONF->mBits.refdiv      = 3    ; // [22 : 21]
    p_PLLCONF->mBits.reserved_23 = 1    ; // [     23]
    p_PLLCONF->mBits.refouten    = 1    ; // [     24]
    p_PLLCONF->mBits.ivco        = 0    ; // [     26]
    p_PLLCONF->mBits.vcoen       = 1    ; // [     27]

    printk("PLLCONF:%x\n",p_PLLCONF->dwValue);
    //----------------------------------------------------------
    p_DIV->mBits.rdiv         = 16   ; // [12 :  3] 
    p_DIV->mBits.ndiv         = 1536 ; // [27 : 13] 

    printk("DIV:%x\n",p_DIV->dwValue);

    
    //----------------------------------------------------------
    p_FDIV->mBits.fdiv        = 0x80000  ; // [27 :  8] 
    p_FDIV->mBits.reserved_0  = 0x70     ; // [ 7 :  0]

    printk("FDIV:%x\n",p_FDIV->dwValue);
    
    //----------------------------------------------------------
    p_STRM->mBits.framecount  = 0x8000000 ; // [27 :  0] 

    printk("STRM:%x\n",p_STRM->dwValue);

    //----------------------------------------------------------
    p_CLK->mBits.mode         = 0    ; // [      0]
    p_CLK->mBits.serclk       = 1    ; // [      1]
    p_CLK->mBits.adcclk       = 0    ; // [      2]
    p_CLK->mBits.fclkin       = 0    ; // [      3]
    p_CLK->mBits.m_cnt        = 1563 ; // [15 :  4]
    p_CLK->mBits.l_cnt        = 256  ; // [27 : 16]
    printk("CLK:%x\n",p_CLK->dwValue);
    //----------------------------------------------------------

}

//-----------------------------------------------------
extern  int gps_spi_write_bytes( unsigned int  len, unsigned int  addr,unsigned int data);
#define Writesnap(x,y)       \
gps_spi_write_bytes(1,x,y)


extern int gps_spi_read_bytes( unsigned int len,unsigned int addr,unsigned int *data);
#define Readsnap(x,y) \
gps_spi_read_bytes(1,x,y)


#define BASE_ADDR 0

void snap_reg_init(void)
{
    unsigned int  value;
 /*   
 printk("##############Test mode XCTL02(0000-0fd8)####################\n");
Writesnap( BASE_ADDR+0x000, 0x0A2A4A43); 
Writesnap( BASE_ADDR+0x004, 0x0000A00A); 

Writesnap( BASE_ADDR+0x030, 0xFF000100); 
Writesnap( BASE_ADDR+0x034, 0x00000003); 
Writesnap( BASE_ADDR+0x008, 0x00003147); 



Readsnap( BASE_ADDR+0x008, &value); 
Readsnap( BASE_ADDR+0x030, &value); 
Readsnap( BASE_ADDR+0x034, &value);



// config the satellite info
Writesnap( BASE_ADDR+0x100, 0x93000000); 
Writesnap( BASE_ADDR+0x140, 0x0000FF31);
Writesnap( BASE_ADDR+0x180, 0x000548C1); 
Writesnap( BASE_ADDR+0x1C0, 0x80007FFE);

Writesnap( BASE_ADDR+0x104,0xe2000000); 
Writesnap( BASE_ADDR+0x144,0x0000FFA5);
Writesnap( BASE_ADDR+0x184,0x0012196A); 
Writesnap( BASE_ADDR+0x1C4,0x80007FFE);

Writesnap( BASE_ADDR+0x108,0x10000000); 
Writesnap( BASE_ADDR+0x148,0x000000EC);
Writesnap( BASE_ADDR+0x188,0x0014E5CF); 
Writesnap( BASE_ADDR+0x1C8,0x80007FFE);

Writesnap( BASE_ADDR+0x10C,0x52000000); 
Writesnap( BASE_ADDR+0x14C,0x0000FF46);
Writesnap( BASE_ADDR+0x18C,0x00011029); 
Writesnap( BASE_ADDR+0x1CC,0x80007FFE);


Writesnap( BASE_ADDR+0x110,0xFD000000); 
Writesnap( BASE_ADDR+0x150,0x0000002E);
Writesnap( BASE_ADDR+0x190,0x001FC42E); 
Writesnap( BASE_ADDR+0x1D0,0x80007FFE);

Writesnap( BASE_ADDR+0x114,0x40000000); 
Writesnap( BASE_ADDR+0x154,0x000000AC);
Writesnap( BASE_ADDR+0x194,0x00038C83); 
Writesnap( BASE_ADDR+0x1D4,0x80007FFE);

Writesnap( BASE_ADDR+0x118,0xF9000000); 
Writesnap( BASE_ADDR+0x158,0x00000099);
Writesnap( BASE_ADDR+0x198,0x0004E36F); 
Writesnap( BASE_ADDR+0x1D8,0x80007FFE);

Writesnap( BASE_ADDR+0x11C,0xD0000000); 
Writesnap( BASE_ADDR+0x15C,0x0000FF46);
Writesnap( BASE_ADDR+0x19C,0x001B730F); 
Writesnap( BASE_ADDR+0x1DC,0x80007FFE);

Writesnap( BASE_ADDR+0x120,0x5A000000); 
Writesnap( BASE_ADDR+0x160,0x0000008A);
Writesnap( BASE_ADDR+0x1A0,0x00047661); 
Writesnap( BASE_ADDR+0x1E0,0x80007FFE);

Writesnap( BASE_ADDR+0x124,0xE9000000);   
Writesnap( BASE_ADDR+0x164,0x00000031);
Writesnap( BASE_ADDR+0x1A4,0x0004EA10); 
Writesnap( BASE_ADDR+0x1E4,0x80007FFE);



//Config other registers
Writesnap( BASE_ADDR+0x020,0x07ED4430); 
Writesnap( BASE_ADDR+0x000,0x0A2A4A43);
Writesnap( BASE_ADDR+0x004,0x0000A00A); 
Writesnap( BASE_ADDR+0x03C,0x00000000);
Writesnap( BASE_ADDR+0x024,0x000002E8); 
Writesnap( BASE_ADDR+0x028,0x00000000);
Writesnap( BASE_ADDR+0x00C,0x0000080C); 
Writesnap( BASE_ADDR+0x088,0x00000000);
Writesnap( BASE_ADDR+0x004,0x0000A1FA);

*/

#ifdef XSC004   
 printk("##############Test mode XSC004(length:4K words!)####################\n");
Writesnap( BASE_ADDR+0x030, 0xFF000010); 
Writesnap( BASE_ADDR+0x034, 0x00000003); 
Writesnap( BASE_ADDR+0x008, 0x000004ED); 



Readsnap( BASE_ADDR+0x008, &value); 
Readsnap( BASE_ADDR+0x030, &value); 
Readsnap( BASE_ADDR+0x034, &value);



// config the satellite info
Writesnap( BASE_ADDR+0x100, 0x93000000); 
Writesnap( BASE_ADDR+0x140, 0x0000FF31);
Writesnap( BASE_ADDR+0x180, 0x000548C1); 
Writesnap( BASE_ADDR+0x1C0, 0x80007FFE);

Writesnap( BASE_ADDR+0x104,0xe2000000); 
Writesnap( BASE_ADDR+0x144,0x0000FFA5);
Writesnap( BASE_ADDR+0x184,0x0012196A); 
Writesnap( BASE_ADDR+0x1C4,0x80007FFE);

Writesnap( BASE_ADDR+0x108,0x10000000); 
Writesnap( BASE_ADDR+0x148,0x000000EC);
Writesnap( BASE_ADDR+0x188,0x0014E5CF); 
Writesnap( BASE_ADDR+0x1C8,0x80007FFE);

Writesnap( BASE_ADDR+0x10C,0x52000000); 
Writesnap( BASE_ADDR+0x14C,0x0000FF46);
Writesnap( BASE_ADDR+0x18C,0x00011029); 
Writesnap( BASE_ADDR+0x1CC,0x80007FFE);


Writesnap( BASE_ADDR+0x110,0xFD000000); 
Writesnap( BASE_ADDR+0x150,0x0000002E);
Writesnap( BASE_ADDR+0x190,0x001FC42E); 
Writesnap( BASE_ADDR+0x1D0,0x80007FFE);

Writesnap( BASE_ADDR+0x114,0x40000000); 
Writesnap( BASE_ADDR+0x154,0x000000AC);
Writesnap( BASE_ADDR+0x194,0x00038C83); 
Writesnap( BASE_ADDR+0x1D4,0x80007FFE);

Writesnap( BASE_ADDR+0x118,0xF9000000); 
Writesnap( BASE_ADDR+0x158,0x00000099);
Writesnap( BASE_ADDR+0x198,0x0004E36F); 
Writesnap( BASE_ADDR+0x1D8,0x80007FFE);

Writesnap( BASE_ADDR+0x11C,0xD0000000); 
Writesnap( BASE_ADDR+0x15C,0x0000FF46);
Writesnap( BASE_ADDR+0x19C,0x001B730F); 
Writesnap( BASE_ADDR+0x1DC,0x80007FFE);

Writesnap( BASE_ADDR+0x120,0x5A000000); 
Writesnap( BASE_ADDR+0x160,0x0000008A);
Writesnap( BASE_ADDR+0x1A0,0x00047661); 
Writesnap( BASE_ADDR+0x1E0,0x80007FFE);

Writesnap( BASE_ADDR+0x124,0xE9000000);   
Writesnap( BASE_ADDR+0x164,0x00000031);
Writesnap( BASE_ADDR+0x1A4,0x0004EA10); 
Writesnap( BASE_ADDR+0x1E4,0x80007FFE);



//Config other registers
Writesnap( BASE_ADDR+0x020,0x07ED4430); 
Writesnap( BASE_ADDR+0x000,0x0A2A4C47);
Writesnap( BASE_ADDR+0x004,0x0000800A); 
Writesnap( BASE_ADDR+0x03C,0x00000000);
Writesnap( BASE_ADDR+0x024,0x00000802); 
Writesnap( BASE_ADDR+0x028,0x00000000);
Writesnap( BASE_ADDR+0x00C,0x0000080C); 
Writesnap( BASE_ADDR+0x088,0x0000007F);
Writesnap( BASE_ADDR+0x004,0x000081FA);

#endif //XSC004


#ifdef XSC005   
 printk("##############Test mode XSC005(I1=1,Q1=1)####################\n");
Writesnap( BASE_ADDR+0x030, 0xFF000060); 
Writesnap( BASE_ADDR+0x034, 0x00000003); 
Writesnap( BASE_ADDR+0x008, 0x00003147); 



Readsnap( BASE_ADDR+0x008, &value); 
Readsnap( BASE_ADDR+0x030, &value); 
Readsnap( BASE_ADDR+0x034, &value);



// config the satellite info
Writesnap( BASE_ADDR+0x100, 0x93000000); 
Writesnap( BASE_ADDR+0x140, 0x0000FF31);
Writesnap( BASE_ADDR+0x180, 0x000548C1); 
Writesnap( BASE_ADDR+0x1C0, 0x80007FFE);

Writesnap( BASE_ADDR+0x104,0xe2000000); 
Writesnap( BASE_ADDR+0x144,0x0000FFA5);
Writesnap( BASE_ADDR+0x184,0x0012196A); 
Writesnap( BASE_ADDR+0x1C4,0x80007FFE);

Writesnap( BASE_ADDR+0x108,0x10000000); 
Writesnap( BASE_ADDR+0x148,0x000000EC);
Writesnap( BASE_ADDR+0x188,0x0014E5CF); 
Writesnap( BASE_ADDR+0x1C8,0x80007FFE);

Writesnap( BASE_ADDR+0x10C,0x52000000); 
Writesnap( BASE_ADDR+0x14C,0x0000FF46);
Writesnap( BASE_ADDR+0x18C,0x00011029); 
Writesnap( BASE_ADDR+0x1CC,0x80007FFE);


Writesnap( BASE_ADDR+0x110,0xFD000000); 
Writesnap( BASE_ADDR+0x150,0x0000002E);
Writesnap( BASE_ADDR+0x190,0x001FC42E); 
Writesnap( BASE_ADDR+0x1D0,0x80007FFE);

Writesnap( BASE_ADDR+0x114,0x40000000); 
Writesnap( BASE_ADDR+0x154,0x000000AC);
Writesnap( BASE_ADDR+0x194,0x00038C83); 
Writesnap( BASE_ADDR+0x1D4,0x80007FFE);

Writesnap( BASE_ADDR+0x118,0xF9000000); 
Writesnap( BASE_ADDR+0x158,0x00000099);
Writesnap( BASE_ADDR+0x198,0x0004E36F); 
Writesnap( BASE_ADDR+0x1D8,0x80007FFE);

Writesnap( BASE_ADDR+0x11C,0xD0000000); 
Writesnap( BASE_ADDR+0x15C,0x0000FF46);
Writesnap( BASE_ADDR+0x19C,0x001B730F); 
Writesnap( BASE_ADDR+0x1DC,0x80007FFE);

Writesnap( BASE_ADDR+0x120,0x5A000000); 
Writesnap( BASE_ADDR+0x160,0x0000008A);
Writesnap( BASE_ADDR+0x1A0,0x00047661); 
Writesnap( BASE_ADDR+0x1E0,0x80007FFE);

Writesnap( BASE_ADDR+0x124,0xE9000000);   
Writesnap( BASE_ADDR+0x164,0x00000031);
Writesnap( BASE_ADDR+0x1A4,0x0004EA10); 
Writesnap( BASE_ADDR+0x1E4,0x80007FFE);



//Config other registers
Writesnap( BASE_ADDR+0x020,0x07ED4430); 
Writesnap( BASE_ADDR+0x000,0x0A2A4A45);
Writesnap( BASE_ADDR+0x004,0x0000800A); 
Writesnap( BASE_ADDR+0x03C,0x00000000);

Writesnap( BASE_ADDR+0x024,0x000002E8); 
Writesnap( BASE_ADDR+0x028,0x00000000);
Writesnap( BASE_ADDR+0x00C,0x0000080C); 
//Writesnap( BASE_ADDR+0x088,0x0000007F);
Writesnap( BASE_ADDR+0x088,0x00000000);
Writesnap( BASE_ADDR+0x004,0x000081FA);

#endif  // XSC005 

#ifdef XESC013   

Writesnap( BASE_ADDR+0x030, 0xFF000040); 
Writesnap( BASE_ADDR+0x034, 0x00000000); 
Writesnap( BASE_ADDR+0x008, 0x00B30071); 



Readsnap( BASE_ADDR+0x008, &value); 
Readsnap( BASE_ADDR+0x030, &value); 
Readsnap( BASE_ADDR+0x034, &value);



// config the satellite info
Writesnap( BASE_ADDR+0x100, 0xF0000000); 
Writesnap( BASE_ADDR+0x140, 0x0000FFBF);
Writesnap( BASE_ADDR+0x180, 0x00031515); 
Writesnap( BASE_ADDR+0x1C0, 0x80007FFE);

Writesnap( BASE_ADDR+0x104,0xe2000000); 
Writesnap( BASE_ADDR+0x144,0x0000FFB3);
Writesnap( BASE_ADDR+0x184,0x000D39B7); 
Writesnap( BASE_ADDR+0x1C4,0x00007FFE);

Writesnap( BASE_ADDR+0x108,0xB8000000); 
Writesnap( BASE_ADDR+0x148,0x0000FFAC);
Writesnap( BASE_ADDR+0x188,0x00080A6F); 
Writesnap( BASE_ADDR+0x1C8,0x80007FFE);

Writesnap( BASE_ADDR+0x10C,0xC0000000); 
Writesnap( BASE_ADDR+0x14C,0x00000099);
Writesnap( BASE_ADDR+0x18C,0x000B0FD8); 
Writesnap( BASE_ADDR+0x1CC,0x00007FFE);


Writesnap( BASE_ADDR+0x110,0x48000000); 
Writesnap( BASE_ADDR+0x150,0x00000053);
Writesnap( BASE_ADDR+0x190,0x000FAE69); 
Writesnap( BASE_ADDR+0x1D0,0x80007FFE);

Writesnap( BASE_ADDR+0x114,0x00000000); 
Writesnap( BASE_ADDR+0x154,0x000000CD);
Writesnap( BASE_ADDR+0x194,0x00061ADF); 
Writesnap( BASE_ADDR+0x1D4,0x80007FFE);

Writesnap( BASE_ADDR+0x118,0x68000000); 
Writesnap( BASE_ADDR+0x158,0x00000006);
Writesnap( BASE_ADDR+0x198,0x000F6EE1); 
Writesnap( BASE_ADDR+0x1D8,0x80007FFE);

Writesnap( BASE_ADDR+0x11C,0xEC000000); 
Writesnap( BASE_ADDR+0x15C,0x0000FFAF);
Writesnap( BASE_ADDR+0x19C,0x000F7057); 
Writesnap( BASE_ADDR+0x1DC,0x80007FFE);

Writesnap( BASE_ADDR+0x120,0xBC000000); 
Writesnap( BASE_ADDR+0x160,0x0000FFBC);
Writesnap( BASE_ADDR+0x1A0,0x0001F494); 
Writesnap( BASE_ADDR+0x1E0,0x80007FFE);

Writesnap( BASE_ADDR+0x124,0xF4000000);   
Writesnap( BASE_ADDR+0x164,0x0000009C);
Writesnap( BASE_ADDR+0x1A4,0x00072112); 
Writesnap( BASE_ADDR+0x1E4,0x80007FFE);

Writesnap( BASE_ADDR+0x128,0x7C000000);   
Writesnap( BASE_ADDR+0x168,0x00000056);
Writesnap( BASE_ADDR+0x1A8,0x00061DEC); 
Writesnap( BASE_ADDR+0x1E8,0x80007FFE);

Writesnap( BASE_ADDR+0x12C,0x44000000);   
Writesnap( BASE_ADDR+0x16C,0x00000110);
Writesnap( BASE_ADDR+0x1AC,0x00032F16); 
Writesnap( BASE_ADDR+0x1EC,0x80007FFE);


//Config other registers
Writesnap( BASE_ADDR+0x020,0x07ED4430); 
Writesnap( BASE_ADDR+0x000,0x3A2A4B41);
Writesnap( BASE_ADDR+0x004,0x0000800B); 
Writesnap( BASE_ADDR+0x03C,0x00000000);
Writesnap( BASE_ADDR+0x024,0x00000277); 
Writesnap( BASE_ADDR+0x028,0x00000000);
Writesnap( BASE_ADDR+0x00C,0x0000080C); 
Writesnap( BASE_ADDR+0x088,0x0000007F);
Writesnap( BASE_ADDR+0x004,0x000081FB);

#endif //XESC013



 
}


