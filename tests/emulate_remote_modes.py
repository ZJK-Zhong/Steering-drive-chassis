"""Run the ARMCC test image with Unicorn, implementing console semihosting only."""
import struct
import sys
from pathlib import Path

image = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(image.parent / "runtime"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_PC

machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0, 0x200000)
machine.mem_map(0x20000000, 0x100000)
with image.open("rb") as source:
    elf = ELFFile(source)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            machine.mem_write(segment["p_vaddr"], segment.data())

output = bytearray()
exited = False
exit_code = None


def words(address, count):
    return struct.unpack("<" + "I" * count, machine.mem_read(address, 4 * count))


def write_console(data):
    output.extend(data)
    sys.stdout.write(data.decode("ascii", errors="replace"))


def semihosting(cpu, interrupt, user_data):
    global exited, exit_code
    if interrupt != 7:
        raise RuntimeError(f"Unexpected ARM exception {interrupt}")
    operation = cpu.reg_read(UC_ARM_REG_R0)
    argument = cpu.reg_read(UC_ARM_REG_R1)
    result = 0
    if operation == 0x01:  # SYS_OPEN: no access to host files.
        name, mode, length = words(argument, 3)
        if bytes(cpu.mem_read(name, length)) != b":tt":
            raise RuntimeError("Test requested a non-console file")
        result = 1 if mode < 4 else 2
    elif operation == 0x03:  # SYS_WRITEC
        write_console(cpu.mem_read(argument, 1))
    elif operation == 0x04:  # SYS_WRITE0
        data = bytearray()
        while len(data) < 4096:
            char = cpu.mem_read(argument + len(data), 1)[0]
            if char == 0:
                break
            data.append(char)
        write_console(data)
    elif operation == 0x05:  # SYS_WRITE
        handle, address, length = words(argument, 3)
        write_console(cpu.mem_read(address, length))
    elif operation == 0x09:  # SYS_ISTTY
        result = 1
    elif operation == 0x15:  # SYS_GET_CMDLINE
        address, size = words(argument, 2)
        cpu.mem_write(address, b"\0")
        cpu.mem_write(argument + 4, struct.pack("<I", 0))
    elif operation == 0x16:  # SYS_HEAPINFO
        address = words(argument, 1)[0]
        cpu.mem_write(address, struct.pack("<4I", 0x20080000, 0x200A0000,
                                          0x200F0000, 0x200D0000))
    elif operation in (0x18, 0x20):  # SYS_EXIT / SYS_EXIT_EXTENDED
        reason, status = (argument, 0) if operation == 0x18 else words(argument, 2)
        exit_code = status if reason == 0x20026 else 1
        exited = True
        cpu.emu_stop()
    elif operation not in (0x02, 0x08, 0x0A, 0x0C, 0x10, 0x13):
        raise RuntimeError(f"Unsupported semihosting operation 0x{operation:x}")
    cpu.reg_write(UC_ARM_REG_R0, result)
    pc = cpu.reg_read(UC_ARM_REG_PC)
    if bytes(cpu.mem_read(pc, 2)) == b"\xab\xbe":
        cpu.reg_write(UC_ARM_REG_PC, (pc + 2) | 1)


stack, reset = words(0, 2)
machine.reg_write(UC_ARM_REG_SP, stack)
machine.hook_add(UC_HOOK_INTR, semihosting)
try:
    machine.emu_start(reset, 0x1FFFFE, timeout=10_000_000, count=50_000_000)
except UcError as error:
    pc = machine.reg_read(UC_ARM_REG_PC)
    raise SystemExit(f"ARM emulator error: {error}; PC=0x{pc:x}, "
                     f"bytes={bytes(machine.mem_read(pc, 8)).hex()}") from error
if not exited or exit_code != 0 or b"PASS: switch modes" not in output:
    raise SystemExit(f"Regression failed: exited={exited}, status={exit_code}, "
                     f"PC=0x{machine.reg_read(UC_ARM_REG_PC):x}")
