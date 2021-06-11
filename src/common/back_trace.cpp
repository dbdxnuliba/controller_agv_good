#include "common/back_trace.h"

#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
// #include <print.h>
#include <signal.h>
#include "common/print.h"


/* 打印调用栈的最大深度 */
#define DUMP_STACK_DEPTH_MAX 16

/* 打印调用栈函数 */
void print_trace()
{
    char buff[64] = {0x00};
    sprintf(buff, "cat /proc/%d/maps", getpid());
    system((const char *) buff);

    //return;
    void *stack_trace[DUMP_STACK_DEPTH_MAX] = {0};
    char **stack_strings = NULL;
    int stack_depth = 0;
    int i = 0;

    /* 获取栈中各层调用函数地址 */
    stack_depth = backtrace(stack_trace, DUMP_STACK_DEPTH_MAX);

    /* 查找符号表将函数调用地址转换为函数名称 */
    stack_strings = (char **)backtrace_symbols(stack_trace, stack_depth);

    if(NULL == stack_strings)
    {
        PRINT_ERROR(" Memory is not enough while dump Stack Trace! \r\n");
        return;
    }

    /* 打印调用栈 */
    PRINT_ERROR(" Stack Trace: \r\n");

    for(i = 0; i < stack_depth; ++i)
    {
        PRINT_ERROR(" [{:2d}] {} \r\n", i, stack_strings[i]);
    }

    /* 获取函数名称时申请的内存需要自行释放 */
    free(stack_strings);
    stack_strings = NULL;

    return;
}





#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
void backtrace(int signo)
{
    char pid_buf[30];
    char name_buf[512];
    char filename[512];

    signal(signo, SIG_IGN); // prevent multiple invocations on same signal
    snprintf(pid_buf, sizeof(pid_buf), "%d", getpid());
    snprintf(filename, sizeof(filename),"/tmp/backtrace.%s", pid_buf);

    name_buf[readlink("/proc/self/exe", name_buf, 511)]=0;
    int child_pid = fork();
    if (!child_pid) {
    fclose(stderr);
    stderr = freopen(filename, "a", stderr);
        dup2(2,1); // child: redirect output to stderr
        fprintf(stdout,"stack trace for %s pid=%s signal=%d\n",name_buf,pid_buf, signo);
        execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread", "-ex", "bt", name_buf, pid_buf, NULL);
    //emcOperatorError(0, "backtrace for %s (pid %s signal %d): gdb failed to start", name_buf, pid_buf, signo);
        abort(); /* If gdb failed to start */
    } else {
    int status;
        waitpid(child_pid, &status,0);
    if (signo == SIGUSR1) {  // continue running after backtrace
        signal(SIGUSR1, backtrace);
        printf("backtrace for %s stored in %s, continuing", name_buf, filename);
        fprintf(stderr, "%s: backtrace stored in %s, continuing\n", name_buf, filename);
    } else {
        // this takes emcmodule.cc:EMC_COMMAND_TIMEOUT seconds to display:
        if (status == 0) // backtrace succeeded
        printf("%s (pid %d) died on signal %d, backtrace stored in %s",
                 name_buf, getpid(), signo, filename);
        fprintf(stderr, "%s exiting\n", name_buf);
        //done = 1;  // signal task to exit main loop
    }
    }
}

void sigsegv_handler(int signo)
{

        PRINT_ERROR("Receive SIGSEGV signal\n");
        PRINT_ERROR("-----call stack-----\n");
        //print_trace();
        backtrace(signo);
        exit(-1);
}

void install_system_signal_monitor()
{
    PRINT_INFO("The code compile date:[{}]:[{}]", __DATE__, __TIME__);

    signal(SIGSEGV, sigsegv_handler);
    signal(SIGFPE, sigsegv_handler);
    signal(SIGUSR1, sigsegv_handler);
}

#if 1
#ifndef _GNU_SOURCE
    #define _GNU_SOURCE
#endif
#include <stdio.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <ucontext.h>


/* 纯C环境下，定义宏NO_CPP_DEMANGLE */
#if (!defined(__cplusplus)) && (!defined(NO_CPP_DEMANGLE))
# define NO_CPP_DEMANGLE
#endif

#ifndef NO_CPP_DEMANGLE
# include <cxxabi.h>
# ifdef __cplusplus
    using __cxxabiv1::__cxa_demangle;
# endif
#endif

#if (defined HAS_ULSLIB)
# include <uls/logger.h>
# define sigsegv_outp(x)	sigsegv_outp(, gx)
#else
# define sigsegv_outp(x, ...) 	fprintf(stderr, x"\n", ##__VA_ARGS__)
#endif

#if (defined (__x86_64__))
# define REGFORMAT   "%016lx"
#elif (defined (__i386__))
# define REGFORMAT   "%08x"
#elif (defined (__arm__))
# define REGFORMAT   "%lx"
#endif

static void print_reg(const ucontext_t *uc)
{
#if (defined (__x86_64__)) || (defined (__i386__))
    int i;
    for (i = 0; i < NGREG; i++) {
        //sigsegv_outp("reg[%02d]: 0x"REGFORMAT, i, uc->uc_mcontext.gregs[i]);
        PRINT_ERROR("reg[{:02d}]: 0x{:x}", i, uc->uc_mcontext.gregs[i]);
    }
#elif (defined (__arm__))
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 0, uc->uc_mcontext.arm_r0);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 1, uc->uc_mcontext.arm_r1);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 2, uc->uc_mcontext.arm_r2);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 3, uc->uc_mcontext.arm_r3);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 4, uc->uc_mcontext.arm_r4);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 5, uc->uc_mcontext.arm_r5);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 6, uc->uc_mcontext.arm_r6);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 7, uc->uc_mcontext.arm_r7);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 8, uc->uc_mcontext.arm_r8);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 9, uc->uc_mcontext.arm_r9);
//    sigsegv_outp("reg[{:02d}]		= 0x"REGFORMAT, 10, uc->uc_mcontext.arm_r10);
//    sigsegv_outp("FP		= 0x"REGFORMAT, uc->uc_mcontext.arm_fp);
//    sigsegv_outp("IP		= 0x"REGFORMAT, uc->uc_mcontext.arm_ip);
//    sigsegv_outp("SP		= 0x"REGFORMAT, uc->uc_mcontext.arm_sp);
//    sigsegv_outp("LR		= 0x"REGFORMAT, uc->uc_mcontext.arm_lr);
//    sigsegv_outp("PC		= 0x"REGFORMAT, uc->uc_mcontext.arm_pc);
//    sigsegv_outp("CPSR		= 0x"REGFORMAT, uc->uc_mcontext.arm_cpsr);
//    sigsegv_outp("Fault Address	= 0x"REGFORMAT, uc->uc_mcontext.fault_address);
//    sigsegv_outp("Trap no		= 0x"REGFORMAT, uc->uc_mcontext.trap_no);
//    sigsegv_outp("Err Code	= 0x"REGFORMAT, uc->uc_mcontext.error_code);
//    sigsegv_outp("Old Mask	= 0x"REGFORMAT, uc->uc_mcontext.oldmask);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 0, uc->uc_mcontext.arm_r0);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 1, uc->uc_mcontext.arm_r1);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 2, uc->uc_mcontext.arm_r2);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 3, uc->uc_mcontext.arm_r3);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 4, uc->uc_mcontext.arm_r4);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 5, uc->uc_mcontext.arm_r5);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 6, uc->uc_mcontext.arm_r6);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 7, uc->uc_mcontext.arm_r7);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 8, uc->uc_mcontext.arm_r8);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 9, uc->uc_mcontext.arm_r9);
    PRINT_ERROR("reg[%02d]		= 0x{:x}", 10, uc->uc_mcontext.arm_r10);
    PRINT_ERROR("FP		= 0x{:x}", uc->uc_mcontext.arm_fp);
    PRINT_ERROR("IP		= 0x{:x}", uc->uc_mcontext.arm_ip);
    PRINT_ERROR("SP		= 0x{:x}", uc->uc_mcontext.arm_sp);
    PRINT_ERROR("LR		= 0x{:x}", uc->uc_mcontext.arm_lr);
    PRINT_ERROR("PC		= 0x{:x}", uc->uc_mcontext.arm_pc);
    PRINT_ERROR("CPSR		= 0x{:x}", uc->uc_mcontext.arm_cpsr);
    PRINT_ERROR("Fault Address	= 0x{:x}", uc->uc_mcontext.fault_address);
    PRINT_ERROR("Trap no		= 0x{:x}", uc->uc_mcontext.trap_no);
    PRINT_ERROR("Err Code	= 0x{:x}", uc->uc_mcontext.error_code);
    PRINT_ERROR("Old Mask	= 0x{:x}", uc->uc_mcontext.oldmask);
#endif
}

static void print_call_link(const ucontext_t *uc)
{
    int i = 0;
    Dl_info	dl_info;

#if (defined (__i386__))
    const void **frame_pointer = (const void **)uc->uc_mcontext.gregs[REG_EBP];
    const void *return_address = (const void *)uc->uc_mcontext.gregs[REG_EIP];
#elif (defined (__x86_64__))
    const void **frame_pointer = (const void **)uc->uc_mcontext.gregs[REG_RBP];
    const void *return_address = (const void *)uc->uc_mcontext.gregs[REG_RIP];
#elif (defined (__arm__))
/* sigcontext_t on ARM:
        unsigned long trap_no;
        unsigned long error_code;
        unsigned long oldmask;
        unsigned long arm_r0;
        ...
        unsigned long arm_r10;
        unsigned long arm_fp;
        unsigned long arm_ip;
        unsigned long arm_sp;
        unsigned long arm_lr;
        unsigned long arm_pc;
        unsigned long arm_cpsr;
        unsigned long fault_address;
*/
    const void **frame_pointer = (const void **)uc->uc_mcontext.arm_fp;
    const void *return_address = (const void *)uc->uc_mcontext.arm_pc;
#endif

    //sigsegv_outp("\nStack trace:");
    PRINT_ERROR("stack trace start:");
    while (return_address) {
        memset(&dl_info, 0, sizeof(Dl_info));
        if (!dladdr((void *)return_address, &dl_info))	break;
        const char *sname = dl_info.dli_sname;
#if (!defined NO_CPP_DEMANGLE)
        int status;
        char *tmp = __cxa_demangle(sname, NULL, 0, &status);
        if (status == 0 && tmp) {
            sname = tmp;
        }
#endif
        /* No: return address <sym-name + offset> (filename) */
//        sigsegv_outp("%02d: %p <%s + %lu> (%s)", ++i, return_address, sname,
//            (unsigned long)return_address - (unsigned long)dl_info.dli_saddr,
//                                                    dl_info.dli_fname);
        PRINT_ERROR("{:2d}: {} <{} + {}> ({})", ++i, return_address, sname,
            (unsigned long)return_address - (unsigned long)dl_info.dli_saddr,
                                                    dl_info.dli_fname);
#if (!defined NO_CPP_DEMANGLE)
        if (tmp)	free(tmp);
#endif
        if (dl_info.dli_sname && !strcmp(dl_info.dli_sname, "main")) break;

        if (!frame_pointer)	break;
#if (defined (__x86_64__)) || (defined (__i386__))
        return_address = frame_pointer[1];
        frame_pointer = (const void **)frame_pointer[0];
#elif (defined (__arm__))
        return_address = frame_pointer[-1];
        frame_pointer = (const void **)frame_pointer[-3];
#endif
    }
    //sigsegv_outp("Stack trace end.");
    PRINT_ERROR("stack trace end.");
}

static void sigsegv_handler(int signo, siginfo_t *info, void *context)
{
//    sigsegv_outp("Segmentation Fault!");
//    sigsegv_outp("info.si_signo = %d", signo);
//    if (info) {
//        sigsegv_outp("info.si_errno = %d", info->si_errno);
//        sigsegv_outp("info.si_code  = %d (%s)", info->si_code,
//            (info->si_code == SEGV_MAPERR) ? "SEGV_MAPERR" : "SEGV_ACCERR");
//        sigsegv_outp("info.si_addr  = %p\n", info->si_addr);
//    }

    PRINT_ERROR("Segmentation Fault!");
    PRINT_ERROR("info.si_signo = {}", signo);
    if (info) {
        PRINT_ERROR("info.si_errno = {}", info->si_errno);
        PRINT_ERROR("info.si_code  = {} ({})", info->si_code,
            (info->si_code == SEGV_MAPERR) ? "SEGV_MAPERR" : "SEGV_ACCERR");
        PRINT_ERROR("info.si_addr  = {}\n", info->si_addr);
    }

    if (context) {
        const ucontext_t *uc = (const ucontext_t *)context;

        print_reg(uc);
        print_call_link(uc);
    }

    _exit(0);
}

#define SETSIG(sa, signo, func, flags)	\
        do {                            \
            sa.sa_sigaction = func;  	\
            sa.sa_flags = flags;        \
            sigemptyset(&sa.sa_mask);   \
            sigaction(signo, &sa, NULL);\
        } while(0)

static void __attribute((constructor)) setup_sigsegv(void)
{
    struct sigaction sa;

    SETSIG(sa, SIGSEGV, sigsegv_handler, SA_SIGINFO);
#if 0
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_sigaction = sigsegv_handler;
    sa.sa_flags = SA_SIGINFO;
    if (sigaction(SIGSEGV, &sa, NULL) < 0) {
        perror("sigaction: ");
    }
#endif
}
#endif
