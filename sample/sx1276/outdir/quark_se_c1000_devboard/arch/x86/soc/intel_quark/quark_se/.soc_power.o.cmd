cmd_arch/x86/soc/intel_quark/quark_se/soc_power.o := /opt/zephyr-sdk//sysroots/x86_64-pokysdk-linux/usr/bin/i586-zephyr-elfiamcu/i586-zephyr-elfiamcu-gcc -Wp,-MD,arch/x86/soc/intel_quark/quark_se/.soc_power.o.d  -nostdinc -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed -I/home/intel_2/zephyr/kernel/include -I/home/intel_2/zephyr/arch/x86/include -I/home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se -I/home/intel_2/zephyr/boards/x86/quark_se_c1000_devboard  -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/include/generated -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/misc/generated/sysgen -include /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx1276/outdir/quark_se_c1000_devboard/include/generated/autoconf.h  -I/home/intel_2/zephyr/ext/hal/qmsi/include -I/home/intel_2/zephyr/ext/hal/qmsi/drivers/include -I/home/intel_2/zephyr/ext/hal/qmsi/soc/quark_se/include/ -I/home/intel_2/zephyr/ext/lib/mraa/include -I/home/intel_2/zephyr/ext/lib/upm/include -I/home/intel_2/zephyr/ext/lib/upm/src/utilities -I/home/intel_2/zephyr/ext/lib/upm/src/sx1276 -I /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include -DKERNEL -D__ZEPHYR__=1 -DENABLE_EXTERNAL_ISR_HANDLING -c -g -xassembler-with-cpp -D_ASMLANGUAGE -march=lakemont -mtune=lakemont -msoft-float -miamcu -DQM_LAKEMONT=1   -I/home/intel_2/zephyr/arch/x86   -I/home/intel_2/zephyr/include/drivers   -I/home/intel_2/zephyr/drivers   -c -o arch/x86/soc/intel_quark/quark_se/soc_power.o /home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se/soc_power.S

source_arch/x86/soc/intel_quark/quark_se/soc_power.o := /home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se/soc_power.S

deps_arch/x86/soc/intel_quark/quark_se/soc_power.o := \
    $(wildcard include/config/sys/power/deep/sleep.h) \
    $(wildcard include/config/debug.h) \
    $(wildcard include/config/soc/watch.h) \
    $(wildcard include/config/bsp/shared/restore/info/ram/addr.h) \
  /home/intel_2/zephyr/include/arch/x86/asm.h \
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
  /home/intel_2/zephyr/include/sections.h \
  /home/intel_2/zephyr/include/section_tags.h \

arch/x86/soc/intel_quark/quark_se/soc_power.o: $(deps_arch/x86/soc/intel_quark/quark_se/soc_power.o)

$(deps_arch/x86/soc/intel_quark/quark_se/soc_power.o):
