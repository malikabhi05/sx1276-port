cmd_src/sx127x.o := /opt/zephyr-sdk//sysroots/x86_64-pokysdk-linux/usr/bin/i586-zephyr-elfiamcu/i586-zephyr-elfiamcu-gcc -Wp,-MD,src/.sx127x.o.d  -nostdinc -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed -I/home/intel_2/zephyr/kernel/include -I/home/intel_2/zephyr/arch/x86/include -I/home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se -I/home/intel_2/zephyr/boards/x86/quark_se_c1000_devboard  -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/misc/generated/sysgen -include /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated/autoconf.h  -I/home/intel_2/zephyr/ext/hal/qmsi/include -I/home/intel_2/zephyr/ext/hal/qmsi/drivers/include -I/home/intel_2/zephyr/ext/hal/qmsi/soc/quark_se/include/ -I/home/intel_2/zephyr/ext/lib/mraa/include -I/home/intel_2/zephyr/ext/lib/upm/include -I/home/intel_2/zephyr/ext/lib/upm/src/utilities -I/home/intel_2/zephyr/ext/lib/upm/src/sx127x -I /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include  -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/src -Isrc -DKERNEL -D__ZEPHYR__=1 -DENABLE_EXTERNAL_ISR_HANDLING -c -g -std=c99 -Wall -Wformat -Wformat-security -D_FORTIFY_SOURCE=2 -Wno-format-zero-length -Wno-main -ffreestanding -Os -fno-asynchronous-unwind-tables -fno-stack-protector -ffunction-sections -fdata-sections -mpreferred-stack-boundary=2 -mno-sse -march=lakemont -mtune=lakemont -msoft-float -miamcu -DQM_LAKEMONT=1 -Wno-unused-but-set-variable -fno-reorder-functions -fno-defer-pop -Wno-pointer-sign -fno-strict-overflow -Werror=implicit-int    -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(sx127x)"  -D"KBUILD_MODNAME=KBUILD_STR(sx127x)" -c -o src/sx127x.o /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/src/sx127x.c

source_src/sx127x.o := /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/src/sx127x.c

deps_src/sx127x.o := \
  /home/intel_2/zephyr/include/zephyr.h \
    $(wildcard include/config/mdef.h) \
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
    $(wildcard include/config/x86.h) \
    $(wildcard include/config/sse.h) \
    $(wildcard include/config/sys/clock/exists.h) \
    $(wildcard include/config/errno.h) \
    $(wildcard include/config/object/monitor.h) \
    $(wildcard include/config/num/mbox/async/msgs.h) \
    $(wildcard include/config/arm.h) \
    $(wildcard include/config/nios2.h) \
    $(wildcard include/config/xtensa.h) \
    $(wildcard include/config/legacy/kernel.h) \
    $(wildcard include/config/multithreading.h) \
    $(wildcard include/config/cplusplus.h) \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include/stddef.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdint.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/_default_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/features.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_newlib_version.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_intsup.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_stdint.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/limits.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/newlib.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/cdefs.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed/limits.h \
  /home/intel_2/zephyr/include/toolchain.h \
  /home/intel_2/zephyr/include/toolchain/gcc.h \
    $(wildcard include/config/isa/thumb.h) \
    $(wildcard include/config/isa/thumb2.h) \
    $(wildcard include/config/isa/arm.h) \
    $(wildcard include/config/riscv32.h) \
    $(wildcard include/config/arc.h) \
  /home/intel_2/zephyr/include/toolchain/common.h \
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
  /home/intel_2/zephyr/include/device.h \
    $(wildcard include/config/kernel/init/priority/default.h) \
    $(wildcard include/config/device/power/management.h) \
  /home/intel_2/zephyr/include/arch/x86/asm_inline.h \
  /home/intel_2/zephyr/include/arch/x86/asm_inline_gcc.h \
    $(wildcard include/config/cmov.h) \
  /home/intel_2/zephyr/include/sys_io.h \
  /home/intel_2/zephyr/include/arch/x86/addr_types.h \
  /home/intel_2/zephyr/include/misc/printk.h \
  /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include/stdarg.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/unistd.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/unistd.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdio.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_ansi.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/stdio.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdlib.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/stdlib.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/string.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/string.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/math.h \
  /home/intel_2/zephyr/ext/lib/upm/include/upm.h \
  /home/intel_2/zephyr/ext/lib/upm/include/upm_types.h \
  /home/intel_2/zephyr/ext/lib/upm/include/upm_math.h \
  /home/intel_2/zephyr/ext/lib/upm/include/upm_platform.h \
    $(wildcard include/config/board/arduino/101.h) \
    $(wildcard include/config/board/arduino/101/sss.h) \
    $(wildcard include/config/board/quark/d2000/crb.h) \
    $(wildcard include/config/board/quark/se/c1000/devboard.h) \
    $(wildcard include/config/board/quark/se/c1000/devboard/ss.h) \
  /home/intel_2/zephyr/ext/lib/upm/src/utilities/upm_utilities.h \
    $(wildcard include/config/stdout/console.h) \
  /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated/version.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/aio.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/common.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/types.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/spi.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/i2c.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/gpio.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/pwm.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/common.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/uart.h \
  /home/intel_2/zephyr/ext/lib/upm/src/sx127x/sx127x.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/gpio.h \
  /home/intel_2/zephyr/ext/lib/mraa/include/mraa/spi.h \

src/sx127x.o: $(deps_src/sx127x.o)

$(deps_src/sx127x.o):