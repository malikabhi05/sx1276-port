cmd_ext/lib/mraa/source/spi.o := /opt/zephyr-sdk//sysroots/x86_64-pokysdk-linux/usr/bin/i586-zephyr-elfiamcu/i586-zephyr-elfiamcu-gcc -Wp,-MD,ext/lib/mraa/source/.spi.o.d  -nostdinc -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed -I/home/intel_2/zephyr/kernel/include -I/home/intel_2/zephyr/arch/x86/include -I/home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se -I/home/intel_2/zephyr/boards/x86/quark_se_c1000_devboard  -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/include/generated -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/misc/generated/sysgen -include /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/include/generated/autoconf.h  -I/home/intel_2/zephyr/ext/hal/qmsi/include -I/home/intel_2/zephyr/ext/hal/qmsi/drivers/include -I/home/intel_2/zephyr/ext/hal/qmsi/soc/quark_se/include/ -I/home/intel_2/zephyr/ext/lib/mraa/include -I/home/intel_2/zephyr/ext/lib/upm/include -I/home/intel_2/zephyr/ext/lib/upm/src/utilities -I/home/intel_2/zephyr/ext/lib/upm/src/sx1276 -I /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include  -I/home/intel_2/zephyr/ext/lib/mraa -Iext/lib/mraa -DKERNEL -D__ZEPHYR__=1 -DENABLE_EXTERNAL_ISR_HANDLING -c -g -std=c99 -Wall -Wformat -Wformat-security -D_FORTIFY_SOURCE=2 -Wno-format-zero-length -Wno-main -ffreestanding -Os -fno-asynchronous-unwind-tables -fno-stack-protector -ffunction-sections -fdata-sections -mpreferred-stack-boundary=2 -mno-sse -march=lakemont -mtune=lakemont -msoft-float -miamcu -DQM_LAKEMONT=1 -Wno-unused-but-set-variable -fno-reorder-functions -fno-defer-pop -Wno-pointer-sign -fno-strict-overflow -Werror=implicit-int   -I/home/intel_2/zephyr/ext/lib/mraa/include    -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(spi)"  -D"KBUILD_MODNAME=KBUILD_STR(spi)" -c -o ext/lib/mraa/source/spi.o /home/intel_2/zephyr/ext/lib/mraa/source/spi.c

source_ext/lib/mraa/source/spi.o := /home/intel_2/zephyr/ext/lib/mraa/source/spi.c

deps_ext/lib/mraa/source/spi.o := \
    $(wildcard include/config/stdout/console.h) \
    $(wildcard include/config/board/arduino/101.h) \
    $(wildcard include/config/board/arduino/101/sss.h) \
    $(wildcard include/config/board/quark/d2000/crb.h) \
    $(wildcard include/config/board/quark/se/c1000/devboard.h) \
    $(wildcard include/config/pinmux/name.h) \
  /home/intel_2/zephyr/include/spi.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdint.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/_default_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/features.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_newlib_version.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_intsup.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_stdint.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include/stddef.h \
  /home/intel_2/zephyr/include/device.h \
    $(wildcard include/config/kernel/init/priority/default.h) \
    $(wildcard include/config/device/power/management.h) \
  /home/intel_2/zephyr/include/toolchain.h \
  /home/intel_2/zephyr/include/toolchain/gcc.h \
    $(wildcard include/config/arm.h) \
    $(wildcard include/config/isa/thumb.h) \
    $(wildcard include/config/isa/thumb2.h) \
    $(wildcard include/config/isa/arm.h) \
    $(wildcard include/config/nios2.h) \
    $(wildcard include/config/riscv32.h) \
    $(wildcard include/config/xtensa.h) \
    $(wildcard include/config/arc.h) \
    $(wildcard include/config/x86.h) \
  /home/intel_2/zephyr/include/toolchain/common.h \
  /home/intel_2/zephyr/include/kernel.h \
    $(wildcard include/config/kernel/debug.h) \
    $(wildcard include/config/coop/enabled.h) \
    $(wildcard include/config/preempt/enabled.h) \
    $(wildcard include/config/num/coop/priorities.h) \
    $(wildcard include/config/num/preempt/priorities.h) \
    $(wildcard include/config/object/tracing.h) \
    $(wildcard include/config/poll.h) \
    $(wildcard include/config/main/stack/size.h) \
    $(wildcard include/config/idle/stack/size.h) \
    $(wildcard include/config/isr/stack/size.h) \
    $(wildcard include/config/system/workqueue/stack/size.h) \
    $(wildcard include/config/init/stacks.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/fp/sharing.h) \
    $(wildcard include/config/sse.h) \
    $(wildcard include/config/sys/clock/exists.h) \
    $(wildcard include/config/errno.h) \
    $(wildcard include/config/object/monitor.h) \
    $(wildcard include/config/num/mbox/async/msgs.h) \
    $(wildcard include/config/legacy/kernel.h) \
    $(wildcard include/config/multithreading.h) \
    $(wildcard include/config/cplusplus.h) \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/limits.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/newlib.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/cdefs.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed/limits.h \
  /home/intel_2/zephyr/include/sections.h \
  /home/intel_2/zephyr/include/section_tags.h \
  /home/intel_2/zephyr/include/atomic.h \
    $(wildcard include/config/atomic/operations/builtin.h) \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/errno.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/errno.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/reent.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_ansi.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/ieeefp.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/lock.h \
  /home/intel_2/zephyr/include/misc/__assert.h \
    $(wildcard include/config/assert.h) \
    $(wildcard include/config/assert/level.h) \
  /home/intel_2/zephyr/include/misc/dlist.h \
  /home/intel_2/zephyr/include/misc/slist.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include/stdbool.h \
  /home/intel_2/zephyr/include/misc/util.h \
    $(wildcard include/config/myfeature.h) \
  /home/intel_2/zephyr/include/kernel_version.h \
  /home/intel_2/zephyr/include/drivers/rand32.h \
  /home/intel_2/zephyr/include/sys_clock.h \
    $(wildcard include/config/sys/clock/hw/cycles/per/sec.h) \
    $(wildcard include/config/sys/clock/ticks/per/sec.h) \
    $(wildcard include/config/timer/reads/its/frequency/at/runtime.h) \
  /home/intel_2/zephyr/include/arch/cpu.h \
  /home/intel_2/zephyr/include/arch/x86/arch.h \
    $(wildcard include/config/int/latency/benchmark.h) \
    $(wildcard include/config/x86/fixed/irq/mapping.h) \
    $(wildcard include/config/sys/power/management.h) \
    $(wildcard include/config/debug/info.h) \
  /home/intel_2/zephyr/include/irq.h \
  /home/intel_2/zephyr/include/arch/x86/irq_controller.h \
    $(wildcard include/config/mvic.h) \
  /home/intel_2/zephyr/include/drivers/sysapic.h \
    $(wildcard include/config/ioapic/num/rtes.h) \
    $(wildcard include/config/eoi/forwarding/bug.h) \
    $(wildcard include/config/loapic/base/address.h) \
    $(wildcard include/config/eoi/formwarding/bug.h) \
  /home/intel_2/zephyr/include/drivers/ioapic.h \
  /home/intel_2/zephyr/include/drivers/loapic.h \
  /home/intel_2/zephyr/include/arch/x86/asm_inline.h \
  /home/intel_2/zephyr/include/arch/x86/asm_inline_gcc.h \
    $(wildcard include/config/cmov.h) \
  /home/intel_2/zephyr/include/sys_io.h \
  /home/intel_2/zephyr/include/arch/x86/addr_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/string.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_ansi.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/string.h \
  /home/intel_2/zephyr/ext/lib/mraa/source/mraa_internal.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/common.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/types.h \
  /home/intel_2/zephyr/ext/lib/mraa/source/mraa_internal_types.h \
    $(wildcard include/config/mraa/pinmux/count.h) \
    $(wildcard include/config/mraa/gpio/count.h) \
    $(wildcard include/config/mraa/pwm/count.h) \
    $(wildcard include/config/mraa/aio/count.h) \
    $(wildcard include/config/mraa/i2c/count.h) \
    $(wildcard include/config/mraa/spi/count.h) \
    $(wildcard include/config/mraa/uart/count.h) \
    $(wildcard include/config/mraa/pin/count.h) \
  /home/intel_2/zephyr/ext/lib/mraa/source/board_config.h \
    $(wildcard include/config/board/quark/se/c1000/devboard/ss.h) \
  /home/intel_2/zephyr/include/adc.h \
  /home/intel_2/zephyr/include/gpio.h \
    $(wildcard include/config/idx.h) \
  /home/intel_2/zephyr/include/i2c.h \
  /home/intel_2/zephyr/include/pinmux.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/malloc.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/malloc.h \
  /home/intel_2/zephyr/include/zephyr.h \
    $(wildcard include/config/mdef.h) \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/spi.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdio.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include/stdarg.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/stdio.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/common.h \

ext/lib/mraa/source/spi.o: $(deps_ext/lib/mraa/source/spi.o)

$(deps_ext/lib/mraa/source/spi.o):