
""" Caches with options for a simple gem5 configuration script
This file contains L1 I/D and L2 caches to be used in the simple
gem5 configuration script.
"""
import m5
from m5.objects import Cache

# Add the common scripts to our path
m5.util.addToPath("../../")

# Some specific options for caches
# For all options see src/mem/cache/BaseCache.py
class L1Cache(Cache):
    """Simple L1 Cache with default values"""
    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    # Empty constructor since we don’t have any parameters to apply to the base L1 cache
    def __init__(self, options=None):
        # Still need to call the super class's. Or gem5’s SimObject attribute finding 
        # function will fail and the result will be “RuntimeError: maximum recursion depth exceeded” 
        # when you try to instantiate the cache object.
        super(L1Cache, self).__init__()
        pass

    def connectCPU(self, cpu):
        # need to define this in a sub-class!
        raise NotImplementedError

    def connectBus(self, bus):
        self.mem_side = bus.cpu_side_ports


class L1ICache(L1Cache):
    """Simple L1 instruction cache with default values"""
    
    def __init__(self, options=None):
        # The constructor of the parent class is empty. So, no need to pass anything 
        # to the constructor of the parent class
        super(L1ICache, self).__init__()
        
        if not options or not options.l1i_size:
            # Use the default size
            self.size = "16kB"
            print(f"L1 Instr Cache Size (default): {self.size}")
            # return to caller just like the way in C/C++.
            return
            
        self.size = options.l1i_size
        print(f"L1 Instr Cache Size: {self.size}")

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port


class L1DCache(L1Cache):
    """Simple L1 data cache with default values"""

    def __init__(self, options=None):
        super(L1DCache, self).__init__()
        
        if not options or not options.l1d_size:
            # Use the default size
            self.size = "64kB"
            print(f"L1 Data Cache Size (default): {self.size}")
            return

        self.size = options.l1d_size
        print(f"L1 Data Cache Size: {self.size}")
        

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port

class L2Cache(Cache):
    """Simple unified L2 Cache with default values"""
    
    # Default parameters
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    def __init__(self, options=None):
        # Error will occur if you use super(L2Cache, self).__init__(options).
        # This is because the parent class does not accept the type of |options| variable and
        # that the constructor of the parent class is non-empty.
        super(L2Cache, self).__init__()
        
        if not options or not options.l2_size:
            # Use the default size
            self.size = "256kB"
            print(f"L2 Cache Size (default): {self.size}")
            return

        self.size = options.l2_size
        print(f"L2 Cache Size: {self.size}")

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports