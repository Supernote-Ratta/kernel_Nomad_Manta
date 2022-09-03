#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/htfy_dbg.h>

// 20191106:需要的时候我们直接通过命令 echo 23 > /sys/module/htfy_dbg/parameters/ht_ebc_dbg_bits 来设置就可以了。
// echo 0 > /sys/module/htfy_dbg/parameters/ht_ebc_dbg_bits
//  cat /sys/module/htfy_dbg/parameters/ht_ebc_dbg_bits
#define DEFUALT_DBG_BITS         0 //EDBG_INIT|EDBG_BUF
// EDBG_USER|IDBG_WACOM|EDBG_BUF|EDBG_AUTO //(0X3F|EDBG_INIT)
//(EDBG_LUT|EDBG_INIT|EDBG_IQRFRAME|EDBG_WARN|EDBG_USER|EDBG_LCDC|EDBG_AUTO|EDBG_POWER)

int ht_ebc_dbg_bits = DEFUALT_DBG_BITS;
module_param(ht_ebc_dbg_bits, int, S_IRUGO | S_IWUSR);
