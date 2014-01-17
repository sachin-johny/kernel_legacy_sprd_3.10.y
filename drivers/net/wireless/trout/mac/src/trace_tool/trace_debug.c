
#include <stdio.h>

#define TROUT_FUNC_ENTER_DBG0 printf("%s: enter\n",__FUNCTION__);
#define TROUT_FUNC_EXIT_DBG01 printf("%s: exit\n",__FUNCTION__);
#define TROUT_DBG2(fmt, arg...)  do \
    { \
        printf(fmt,##arg); \
    } while(0)
    
        /*printf("%s: debug ",fmt,__FUNCTION__,##arg);*/

/*        printf(fmt, ##arg); \*/

void test1(void);
int *test2(int aa, //444
           int bb, /*fsjdkl*/
           int cc);
int *test3(int aa, //444
           int bb, /*fsjdkl*/
           int cc);

void main(void)
{
    int a = 111;
    char b[] = "sss\0";
    unsigned int c = 333;
    char d = 0x4;

    if(1)
    {
#ifdef WWW
        TROUT_FUNC_ENTER_DBG0
    }
#else
        TROUT_FUNC_EXIT_DBG01
}
#endif
    TROUT_DBG2("%s: testName1 = %d,testName02=%s ,$testName03 =%p, $testName4= %u,"
        "lastName = %x\n",__FUNCTION__,a,b,&a,c,d);
    TROUT_FUNC_EXIT_DBG01
    test1();
    test2(11,22,33);
}

void test1(void)
{
    int a = 111;
    char b[] = "sss\0";
    unsigned int c = 333;
    char d = 0x4;
    
        #ifdef aaa
            char e[5] = {0x11,0x22,0x33,0x44,0x55};
        #endif
#ifdef WWW
        if(1)
        {
            #ifdef aaa
                char e[5] = {0x11,0x22,0x33,0x44,0x55};
            #endif
            TROUT_FUNC_ENTER_DBG0
#else
if(1){
        TROUT_FUNC_EXIT_DBG01
#endif
}
    TROUT_FUNC_ENTER_DBG0
    TROUT_DBG2("%s: testName1 = %d,testName02=%s ,$testName03 =%p, $testName4= %u,"
        "lastName = %x\n",__FUNCTION__,a,b,&a,c,d);
    TROUT_FUNC_EXIT_DBG01
}


int *test2(int aa, //444
           int bb, /*fsjdkl*/
           int cc) { TROUT_FUNC_ENTER_DBG0
    TROUT_FUNC_EXIT_DBG01
}

int *test3(int aa, //444
           int bb, /*fsjdkl*/
           int cc) { printf("test3\n"); printf("name = }fileName\n");
    TROUT_FUNC_ENTER_DBG0
    TROUT_FUNC_EXIT_DBG01
}

