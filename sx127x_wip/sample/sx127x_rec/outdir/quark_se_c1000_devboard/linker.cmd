MEMORY
    {
    ROM (rx) : ORIGIN = 0x40030000, LENGTH = 144*1K
    RAM (wx) : ORIGIN = 0xA8006400, LENGTH = 55*1K - 0x20
    BSP_SHARED_GDT_RAM (rw) : ORIGIN = 0xA8013FE0,
                              LENGTH = 0x20
    IDT_LIST : ORIGIN = 2K, LENGTH = 2K
    }
  OUTPUT_FORMAT("elf32-iamcu")
  OUTPUT_ARCH(iamcu:intel)
SECTIONS
 {

 _image_rom_start = 0x40030000;
 _image_text_start = 0x40030000;
 text () :
 {
 . = 0x0;
 *(.text_start)
 *(".text_start.*")
 *(.text)
 *(".text.*")
 *(.gnu.linkonce.t.*)
 *(.eh_frame)
 *(.init)
 *(.fini)
 *(.eini)
 KEEP(*(.openocd_dbg))
 KEEP(*(".openocd_dbg.*"))

 } > ROM
 _image_text_end = .;
 devconfig () :
 {
  __devconfig_start = .;
  *(".devconfig.*")
  KEEP(*(SORT(".devconfig*")))
  __devconfig_end = .;
 } > ROM
 net_l2 () :
 {
  __net_l2_start = .;
  *(".net_l2.init")
  KEEP(*(SORT(".net_l2.init*")))
  __net_l2_end = .;
 } > ROM
 rodata () :
 {
 *(.rodata)
 *(".rodata.*")
 *(.gnu.linkonce.r.*)
 . = ALIGN(8);
 _idt_base_address = .;
 KEEP(*(staticIdt))
 . = ALIGN(4);
 _irq_to_interrupt_vector = .;
 KEEP(*(irq_int_vector_map))

 } > ROM
 _image_rom_end = .;
 __data_rom_start = ALIGN(4);


 datas () : ALIGN_WITH_INPUT
 {

 _image_ram_start = .;
 __data_ram_start = .;
 *(.top_of_image_ram)
 *(.top_of_image_ram.*)
 *(.data)
 *(".data.*")
 . = ALIGN(4);
 } > RAM AT> ROM
 initlevel () : ALIGN_WITH_INPUT
 {
  __device_init_start = .; __device_PRE_KERNEL_1_start = .; KEEP(*(SORT(.init_PRE_KERNEL_1[0-9]))); KEEP(*(SORT(.init_PRE_KERNEL_1[1-9][0-9]))); __device_PRE_KERNEL_2_start = .; KEEP(*(SORT(.init_PRE_KERNEL_2[0-9]))); KEEP(*(SORT(.init_PRE_KERNEL_2[1-9][0-9]))); __device_POST_KERNEL_start = .; KEEP(*(SORT(.init_POST_KERNEL[0-9]))); KEEP(*(SORT(.init_POST_KERNEL[1-9][0-9]))); __device_APPLICATION_start = .; KEEP(*(SORT(.init_APPLICATION[0-9]))); KEEP(*(SORT(.init_APPLICATION[1-9][0-9]))); __device_PRIMARY_start = .; KEEP(*(SORT(.init_PRIMARY[0-9]))); KEEP(*(SORT(.init_PRIMARY[1-9][0-9]))); __device_SECONDARY_start = .; KEEP(*(SORT(.init_SECONDARY[0-9]))); KEEP(*(SORT(.init_SECONDARY[1-9][0-9]))); __device_NANOKERNEL_start = .; KEEP(*(SORT(.init_NANOKERNEL[0-9]))); KEEP(*(SORT(.init_NANOKERNEL[1-9][0-9]))); __device_MICROKERNEL_start = .; KEEP(*(SORT(.init_MICROKERNEL[0-9]))); KEEP(*(SORT(.init_MICROKERNEL[1-9][0-9]))); __device_init_end = .;
 } > RAM AT> ROM
 initlevel_error () : ALIGN_WITH_INPUT
 {
  KEEP(*(SORT(.init_[_A-Z0-9]*)))
 }
 ASSERT(SIZEOF(initlevel_error) == 0, "Undefined initialization levels used.")
 initshell () : ALIGN_WITH_INPUT
 {
  __shell_cmd_start = .; KEEP(*(".shell_*")); __shell_cmd_end = .;
 } > RAM AT> ROM
 _static_thread_area () : ALIGN_WITH_INPUT
 {
  _static_thread_data_list_start = .;
  KEEP(*(SORT("._static_thread_data.static.*")))
  _static_thread_data_list_end = .;
 } > RAM AT> ROM
 _k_timer_area () : ALIGN_WITH_INPUT
 {
  _k_timer_list_start = .;
  KEEP(*(SORT("._k_timer.static.*")))
  _k_timer_list_end = .;
 } > RAM AT> ROM
 _k_mem_slab_area () : ALIGN_WITH_INPUT
 {
  _k_mem_slab_list_start = .;
  KEEP(*(SORT("._k_mem_slab.static.*")))
  _k_mem_slab_list_end = .;
 } > RAM AT> ROM
 _k_mem_pool_area () : ALIGN_WITH_INPUT
 {
  KEEP(*(SORT("._k_memory_pool.struct*")))
  _k_mem_pool_list_start = .;
  KEEP(*(SORT("._k_mem_pool.static.*")))
  _k_mem_pool_list_end = .;
 } > RAM AT> ROM
 _k_sem_area () : ALIGN_WITH_INPUT
 {
  _k_sem_list_start = .;
  KEEP(*(SORT("._k_sem.static.*")))
  _k_sem_list_end = .;
 } > RAM AT> ROM
 _k_mutex_area () : ALIGN_WITH_INPUT
 {
  _k_mutex_list_start = .;
  KEEP(*(SORT("._k_mutex.static.*")))
  _k_mutex_list_end = .;
 } > RAM AT> ROM
 _k_alert_area () : ALIGN_WITH_INPUT
 {
  _k_alert_list_start = .;
  KEEP(*(SORT("._k_alert.static.*")))
  _k_alert_list_end = .;
 } > RAM AT> ROM
 _k_fifo_area () : ALIGN_WITH_INPUT
 {
  _k_fifo_list_start = .;
  KEEP(*(SORT("._k_fifo.static.*")))
  _k_fifo_list_end = .;
 } > RAM AT> ROM
 _k_lifo_area () : ALIGN_WITH_INPUT
 {
  _k_lifo_list_start = .;
  KEEP(*(SORT("._k_lifo.static.*")))
  _k_lifo_list_end = .;
 } > RAM AT> ROM
 _k_stack_area () : ALIGN_WITH_INPUT
 {
  _k_stack_list_start = .;
  KEEP(*(SORT("._k_stack.static.*")))
  _k_stack_list_end = .;
 } > RAM AT> ROM
 _k_msgq_area () : ALIGN_WITH_INPUT
 {
  _k_msgq_list_start = .;
  KEEP(*(SORT("._k_msgq.static.*")))
  _k_msgq_list_end = .;
 } > RAM AT> ROM
 _k_mbox_area () : ALIGN_WITH_INPUT
 {
  _k_mbox_list_start = .;
  KEEP(*(SORT("._k_mbox.static.*")))
  _k_mbox_list_end = .;
 } > RAM AT> ROM
 _k_pipe_area () : ALIGN_WITH_INPUT
 {
  _k_pipe_list_start = .;
  KEEP(*(SORT("._k_pipe.static.*")))
  _k_pipe_list_end = .;
 } > RAM AT> ROM
 _k_task_list () : ALIGN_WITH_INPUT
 {
  _k_task_list_start = .;
  *(._k_task_list.public.*)
  *(._k_task_list.private.*)
  _k_task_list_idle_start = .;
  *(._k_task_list.idle.*)
  KEEP(*(SORT("._k_task_list*")))
  _k_task_list_end = .;
 } > RAM AT> ROM
 _k_event_list () : ALIGN_WITH_INPUT
 {
  _k_event_list_start = .;
  *(._k_event_list.event.*)
  KEEP(*(SORT("._k_event_list*")))
  _k_event_list_end = .;
 } > RAM AT> ROM
 _k_memory_pool () : ALIGN_WITH_INPUT
 {
  *(._k_memory_pool.struct*)
  KEEP(*(SORT("._k_memory_pool.struct*")))
  _k_mem_pool_start = .;
  *(._k_memory_pool.*)
  KEEP(*(SORT("._k_memory_pool*")))
  _k_mem_pool_end = .;
 } > RAM AT> ROM
 net_if () : ALIGN_WITH_INPUT
 {
  __net_if_start = .;
  *(".net_if.*")
  KEEP(*(SORT(".net_if*")))
  __net_if_end = .;
 } > RAM AT> ROM
 net_l2_data () : ALIGN_WITH_INPUT
 {
  __net_l2_data_start = .;
  *(".net_l2.data")
  KEEP(*(SORT(".net_l2.data*")))
  __net_l2_data_end = .;
 } > RAM AT> ROM
 __data_ram_end = .;
 bss (NOLOAD ) :
 {
 . = ALIGN(4);
 __bss_start = .;
 *(.bss)
 *(".bss.*")
 *(COMMON)
 . = ALIGN(4);
 __bss_end = .;

 } > RAM
   AT > RAM
 noinit (NOLOAD ) :
 {
 *(.noinit)
 *(".noinit.*")
 *(.bottom_of_image_ram)
 *(.bottom_of_image_ram.*)
 } > RAM
 _image_ram_end = .;
 _end = .;
 . = ALIGN(((4) << 10));
 __bss_num_words = (__bss_end - __bss_start) >> 2;

 intList () :
 {
 KEEP(*(.spurIsr))
 KEEP(*(.spurNoErrIsr))
 __INT_LIST_START__ = .;
 LONG((__INT_LIST_END__ - __INT_LIST_START__) / 0x14)
 KEEP(*(.intList))
 KEEP(*(.gnu.linkonce.intList.*))
 __INT_LIST_END__ = .;
 } > IDT_LIST
 }
__data_size = (__data_ram_end - __data_ram_start);
__data_num_words = (__data_size + 3) >> 2;
