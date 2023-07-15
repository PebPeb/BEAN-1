
MEMORY
{
  memory_region (rwx) : ORIGIN = 0x00000000, LENGTH = 256
}

SECTIONS
{
  .text :
  {
    *(.text)
  } > memory_region

  .data :
  {
    *(.data)
    *(.bss)
  } > memory_region
}
