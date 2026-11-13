#include <stddef.h>

// Hỗ trợ lệnh xóa cấu trúc khi firmware không liên kết thư viện C.
void *memset(void *destination, int value, size_t count)
{
    volatile unsigned char *buffer = (volatile unsigned char *)destination;
    while (count > 0U)
    {
        *buffer++ = (unsigned char)value;
        --count;
    }
    return destination;
}

typedef void (*init_function_t)(void);

extern init_function_t __preinit_array_start[];
extern init_function_t __preinit_array_end[];
extern init_function_t __init_array_start[];
extern init_function_t __init_array_end[];

// Startup gọi hàm này để chạy các hàm khởi tạo khi liên kết bằng -nostdlib.
void __libc_init_array(void)
{
    init_function_t *entry;

    for (entry = __preinit_array_start; entry != __preinit_array_end; ++entry)
    {
        (*entry)();
    }
    for (entry = __init_array_start; entry != __init_array_end; ++entry)
    {
        (*entry)();
    }
}
