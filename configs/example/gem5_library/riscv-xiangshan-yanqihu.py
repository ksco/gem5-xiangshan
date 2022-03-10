import m5
from m5.objects import Root

from gem5.utils.requires import requires
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.classic.caches.l1dcache import L1DCache
from gem5.components.cachehierarchies.classic.caches.l1icache import L1ICache
from gem5.components.cachehierarchies.classic.caches.l2cache import L2Cache
from gem5.components.cachehierarchies.classic.caches.mmu_cache import MMUCache
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.simple_core import SimpleCore
from gem5.components.cachehierarchies.classic.\
    private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.resources.resource import Resource
from gem5.simulate.simulator import Simulator

from m5.objects import Cache, L2XBar, BaseXBar, SystemXBar, BadAddr, Port

from m5.objects.RiscvMMU import RiscvMMU
from m5.objects.RiscvTLB import RiscvTLB

from gem5.runtime import get_runtime_isa

"""
This script shows an example of running a full system RISCV simple Linux boot
simulation using the gem5 library. This simulation uses the micro-architecture
of Xiangshan Yanqihu, based on:
https://github.com/OpenXiangShan/XiangShan/raw/master/images/xs-arch-simple.svg

Usage
-----

```
scons build/RISCV/gem5.opt
./build/RISCV/gem5.opt \
    configs/example/gem5_library/riscv-xiangshan-yanqihu.py
```
"""

class YanqihuCore(SimpleCore):
    def __init__(self, core_id: int):
        super().__init__(cpu_type=CPUTypes.O3, core_id=core_id)

        self.core.mmu = RiscvMMU()

        # TODO: Yanqihu has a 2 level TLB,
        # which is currently not supported by RISC-V ISA.
        dtb = RiscvTLB(entry_type="data")
        dtb.size = 4096

        itb = RiscvTLB(entry_type="instruction")
        itb.size = 32

        self.core.mmu.dtb = dtb
        self.core.mmu.itb = itb
        self.core.numPhysIntRegs = 160
        self.core.numPhysFloatRegs = 160
        self.core.numROBEntries = 192
        self.core.LQEntries = 64
        self.core.SQEntries = 48

class YanqihuProcessor(SimpleProcessor):
    def _create_cores(self, cpu_type: CPUTypes, num_cores: int):
        return [
            YanqihuCore(core_id=i) for i in range(num_cores)
        ]

class YanqihuPrivateL1PrivateL2CacheHierarchy(
    PrivateL1PrivateL2CacheHierarchy,
):
    def __init__(
        self,
        l1d_size: str,
        l1i_size: str,
        l2_size: str,
        iptw_size: str,
        dptw_size: str,
    ) -> None:
        super().__init__(l1d_size, l1i_size, l2_size)
        self._iptw_size = iptw_size
        self._dptw_size = dptw_size

    def incorporate_cache(self, board: AbstractBoard) -> None:

        # Set up the system port for functional access from the simulator.
        board.connect_system_port(self.membus.cpu_side_ports)

        for cntr in board.get_memory().get_memory_controllers():
            cntr.port = self.membus.mem_side_ports

        self.l1icaches = [
            L1ICache(size=self._l1i_size)
            for i in range(board.get_processor().get_num_cores())
        ]
        self.l1dcaches = [
            L1DCache(size=self._l1d_size)
            for i in range(board.get_processor().get_num_cores())
        ]
        self.l2buses = [
            L2XBar() for i in range(board.get_processor().get_num_cores())
        ]
        self.l2caches = [
            L2Cache(size=self._l2_size)
            for i in range(board.get_processor().get_num_cores())
        ]
        # ITLB Page walk caches
        self.iptw_caches = [
            MMUCache(size=self._iptw_size)
            for _ in range(board.get_processor().get_num_cores())
        ]
        # DTLB Page walk caches
        self.dptw_caches = [
            MMUCache(size=self._dptw_size)
            for _ in range(board.get_processor().get_num_cores())
        ]

        if board.has_coherent_io():
            self._setup_io_cache(board)

        for i, cpu in enumerate(board.get_processor().get_cores()):

            cpu.connect_icache(self.l1icaches[i].cpu_side)
            cpu.connect_dcache(self.l1dcaches[i].cpu_side)

            self.l1icaches[i].mem_side = self.l2buses[i].cpu_side_ports
            self.l1dcaches[i].mem_side = self.l2buses[i].cpu_side_ports
            self.iptw_caches[i].mem_side = self.l2buses[i].cpu_side_ports
            self.dptw_caches[i].mem_side = self.l2buses[i].cpu_side_ports

            self.l2buses[i].mem_side_ports = self.l2caches[i].cpu_side

            self.membus.cpu_side_ports = self.l2caches[i].mem_side

            cpu.connect_walker_ports(
                self.iptw_caches[i].cpu_side, self.dptw_caches[i].cpu_side,
            )

            if get_runtime_isa() == ISA.X86:
                int_req_port = self.membus.mem_side_ports
                int_resp_port = self.membus.cpu_side_ports
                cpu.connect_interrupt(int_req_port, int_resp_port)
            else:
                cpu.connect_interrupt()


requires(isa_required=ISA.RISCV)

cache_hierarchy = YanqihuPrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB",
    l1i_size="128KiB",
    l2_size="1024KiB",
    iptw_size="16KiB",

    # TODO: Data page table walker should directly connect to L2 cache. But it
    # looks like gem5 will segfault by doing this.
    dptw_size="8KiB",
)

processor = YanqihuProcessor(cpu_type=CPUTypes.O3, num_cores=1)
memory = DualChannelDDR4_2400(size="8GB")
board = RiscvBoard(
    clk_freq="1.3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy
)

board.set_kernel_disk_workload(
    kernel=Resource(
        "riscv-bootloader-vmlinux-5.10",
    ),
    disk_image=Resource(
        "riscv-disk-img",
    ),
)

simulator = Simulator(board=board)
print("Beginning simulation!")

simulator.run()