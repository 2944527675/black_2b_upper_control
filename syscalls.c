/**
  ******************************************************************************
  * @file    syscalls.c
  * @brief   Minimal newlib stubs: printf() goes to the log UART.
  ******************************************************************************
  */
#include <errno.h>
#include <sys/stat.h>

#include "main.h"

extern UART_HandleTypeDef hconsole;

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&hconsole, (uint8_t *)ptr, (uint16_t)len, 1000U);
    return len;
}

int _read(int fd, char *ptr, int len)
{
    (void)fd;
    (void)ptr;
    (void)len;
    return 0;
}

int _close(int fd)
{
    (void)fd;
    return -1;
}

int _lseek(int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;
    return 0;
}

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

extern uint8_t _end;   /* end of .bss, from the linker script */
extern uint8_t _estack;
#define HEAP_LIMIT 0x2000U /* 8 KB heap ceiling */

void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap = NULL;
    uint8_t *prev;

    if (heap == NULL) {
        heap = &_end;
    }
    if ((heap + incr) > (&_estack - HEAP_LIMIT)) {
        errno = ENOMEM;
        return (void *)-1;
    }
    prev = heap;
    heap += incr;
    return prev;
}
