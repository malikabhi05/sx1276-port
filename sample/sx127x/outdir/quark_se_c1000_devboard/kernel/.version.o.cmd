cmd_kernel/version.o := /opt/zephyr-sdk//sysroots/x86_64-pokysdk-linux/usr/bin/i586-zephyr-elfiamcu/i586-zephyr-elfiamcu-gcc -Wp,-MD,kernel/.version.o.d  -nostdinc -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include -isystem /opt/zephyr-sdk/sysroots/x86_64-pokysdk-linux/usr/lib/i586-zephyr-elfiamcu/gcc/i586-zephyr-elfiamcu/6.2.0/include-fixed -I/home/intel_2/zephyr/kernel/include -I/home/intel_2/zephyr/arch/x86/include -I/home/intel_2/zephyr/arch/x86/soc/intel_quark/quark_se -I/home/intel_2/zephyr/boards/x86/quark_se_c1000_devboard  -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/include -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated -I/home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/misc/generated/sysgen -include /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated/autoconf.h  -I/home/intel_2/zephyr/ext/hal/qmsi/include -I/home/intel_2/zephyr/ext/hal/qmsi/drivers/include -I/home/intel_2/zephyr/ext/hal/qmsi/soc/quark_se/include/ -I/home/intel_2/zephyr/ext/lib/mraa/include -I/home/intel_2/zephyr/ext/lib/upm/include -I/home/intel_2/zephyr/ext/lib/upm/src/utilities -I/home/intel_2/zephyr/ext/lib/upm/src/sx127x -I /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include  -I/home/intel_2/zephyr/kernel -Ikernel -DKERNEL -D__ZEPHYR__=1 -DENABLE_EXTERNAL_ISR_HANDLING -c -g -std=c99 -Wall -Wformat -Wformat-security -D_FORTIFY_SOURCE=2 -Wno-format-zero-length -Wno-main -ffreestanding -Os -fno-asynchronous-unwind-tables -fno-stack-protector -ffunction-sections -fdata-sections -mpreferred-stack-boundary=2 -mno-sse -march=lakemont -mtune=lakemont -msoft-float -miamcu -DQM_LAKEMONT=1 -Wno-unused-but-set-variable -fno-reorder-functions -fno-defer-pop -Wno-pointer-sign -fno-strict-overflow -Werror=implicit-int   -I/home/intel_2/zephyr/kernel/include    -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(version)"  -D"KBUILD_MODNAME=KBUILD_STR(version)" -c -o kernel/version.o /home/intel_2/zephyr/kernel/version.c

source_kernel/version.o := /home/intel_2/zephyr/kernel/version.c

deps_kernel/version.o := \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/stdint.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/machine/_default_types.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/features.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/_newlib_version.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_intsup.h \
  /opt/zephyr-sdk//sysroots/iamcu-zephyr-elfiamcu/usr/include/sys/_stdint.h \
  /home/intel_2/zephyr/ext/lib/upm/samples/upm/sx127x/outdir/quark_se_c1000_devboard/include/generated/version.h \

kernel/version.o: $(deps_kernel/version.o)

$(deps_kernel/version.o):
