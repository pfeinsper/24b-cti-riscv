-- ****************************************************************************
-- *  Copyright (c) 2021-2023 by Michael Fischer (www.emb4fun.de)
-- *  All rights reserved.
-- *
-- *  Redistribution and use in source and binary forms, with or without
-- *  modification, are permitted provided that the following conditions
-- *  are met:
-- *
-- *  1. Redistributions of source code must retain the above copyright
-- *     notice, this list of conditions and the following disclaimer.
-- *
-- *  2. Redistributions in binary form must reproduce the above copyright
-- *     notice, this list of conditions and the following disclaimer in the
-- *     documentation and/or other materials provided with the distribution.
-- *
-- *  3. Neither the name of the author nor the names of its contributors may
-- *     be used to endorse or promote products derived from this software
-- *     without specific prior written permission.
-- *
-- *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
-- *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
-- *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
-- *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
-- *  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
-- *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
-- *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
-- *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
-- *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
-- *  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- *  SUCH DAMAGE.
-- ****************************************************************************

-- ****************************************************************************
-- *  DEFINE: Library                                                         *
-- ****************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.ALL;

library neorv32;
use neorv32.neorv32_package.all;


-- ****************************************************************************
-- *  DEFINE: Entity                                                          *
-- ****************************************************************************

entity top is
   port (
		sys_clk_port: out std_logic;
      --
      -- Input clock
      --
      CLOCK_50    : in  std_logic;

      --
      -- User LEDs
      --
      LED         : out std_logic_vector(7 downto 0);

      --
      -- Keys
      --
      KEY         : in  std_logic_vector(1 downto 0);

      --
      -- Switches
      --
      SW         : in  std_logic_vector(3 downto 0);

      --
      -- UART
      --
      UART0_TXD   : out std_logic;
      UART0_RXD   : in  std_logic;
		UART1_TXD   : out std_logic;
      UART1_RXD   : in  std_logic;
		

		--
		-- PWM   
      --
		PWM          : out std_logic_vector(3 downto 0);

		--
      -- GPIO
      --
		GPIO_i	    : in std_logic_vector(12 downto 0);
		GPIO_o		 :	out std_logic_vector(12 downto 0);

      HALL_i   : in std_logic;

      HALL_SENSOR : in std_logic_vector(2 downto 0);
		
		FPGA_RESET_N : in std_logic
   );
end entity top;


-- ****************************************************************************
-- *  DEFINE: Architecture                                                    *
-- ****************************************************************************

architecture syn of top is

   --------------------------------------------------------
   -- Define all constants here
   --------------------------------------------------------

   constant CLOCK_FREQUENCY   : natural := 100000000;    -- clock frequency of clk_i in Hz
   constant MEM_INT_IMEM_SIZE : natural := 64*1024;      -- size of processor-internal instruction memory in bytes
   constant MEM_INT_DMEM_SIZE : natural := 32*1024;      -- size of processor-internal data memory in bytes

   --------------------------------------------------------
   -- Define all components which are included here
   --------------------------------------------------------

   --
   -- Counter
   --
   component counter
      port (
         rst         : in std_logic;
         clk         : in std_logic;
         signal_in   : in std_logic;
         counter : out std_logic_vector(31 downto 0)
      );
   end component counter;

   component hall_sector
      port (
         rst : in std_logic;
         clk : in std_logic;
         signal_in : in std_logic_vector(2 downto 0);
         sector : out std_logic_vector(2 downto 0)
      );
   end component hall_sector;

   --
   -- PLL
   --
   component pll_sys
      port (
         inclk0 : in  std_logic := '0';
         c0     : out std_logic;
         c1     : out std_logic;
         locked : out std_logic
      );
   end component pll_sys;

   --
   -- neorv32 top
   --
   component neorv32_top is
     generic (
       -- General --
       CLOCK_FREQUENCY              : natural;           -- clock frequency of clk_i in Hz
       HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID
		 JEDEC_ID                     : std_ulogic_vector(10 downto 0) := "00000000000"; -- vendor's JEDEC ID
       INT_BOOTLOADER_EN            : boolean := false;  -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

       -- On-Chip Debugger (OCD) --
       ON_CHIP_DEBUGGER_EN          : boolean := false;  -- implement on-chip debugger
       DM_LEGACY_MODE               : boolean := false;                              -- debug module spec version: false = v1.0, true = v0.13

       -- RISC-V CPU Extensions --
       CPU_EXTENSION_RISCV_A        : boolean := false;                              -- implement atomic memory operations extension?
       CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
       CPU_EXTENSION_RISCV_C        : boolean := false;  -- implement compressed extension?
       CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
       CPU_EXTENSION_RISCV_M        : boolean := false;  -- implement mul/div extension?
       CPU_EXTENSION_RISCV_U        : boolean := false;  -- implement user mode extension?
       CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
       CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
		 CPU_EXTENSION_RISCV_Zicond 	: boolean := false;       -- implement integer conditional operations?
       CPU_EXTENSION_RISCV_Zihpm    : boolean := false;  -- implement hardware performance monitors?
       CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
       CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;  -- implement custom (instr.) functions unit?
		 
       -- Tuning Options --
       FAST_MUL_EN                  : boolean := false;  -- use DSPs for M extension's multiplier
       FAST_SHIFT_EN                : boolean := false;  -- use barrel shifter for shift operations
		 REGFILE_HW_RST             	: boolean := false;  -- implement full hardware reset for register file

       -- Physical Memory Protection (PMP) --
       PMP_NUM_REGIONS              : natural range 0 to 16 := 0;                    -- number of regions (0..16)
       PMP_MIN_GRANULARITY          : natural := 4;      -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
		 PMP_TOR_MODE_EN            	: boolean := true;        -- implement TOR mode
		 PMP_NAP_MODE_EN            	: boolean := true;        -- implement NAPOT/NA4 modes

       -- Hardware Performance Monitors (HPM) --
       HPM_NUM_CNTS                 : natural range 0 to 13 := 0;                    -- number of implemented HPM counters (0..13)
       HPM_CNT_WIDTH                : natural range 0 to 64 := 40;                   -- total size of HPM counters (0..64)

       -- Internal Instruction memory (IMEM) --
       MEM_INT_IMEM_EN              : boolean := false;  -- implement processor-internal instruction memory
       MEM_INT_IMEM_SIZE            : natural := 16*1024;                            -- size of processor-internal instruction memory in bytes (use a power of 2)

       -- Internal Data memory (DMEM) --
       MEM_INT_DMEM_EN              : boolean := false;  -- implement processor-internal data memory
       MEM_INT_DMEM_SIZE            : natural := 8*1024;                             -- size of processor-internal data memory in bytes (use a power of 2)

       -- Internal Instruction Cache (iCACHE) --
       ICACHE_EN                    : boolean := false;  -- implement instruction cache
       ICACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- i-cache: number of blocks (min 1), has to be a power of 2
       ICACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- i-cache: block size in bytes (min 4), has to be a power of 2

       -- Internal Data Cache (dCACHE) --
       DCACHE_EN                    : boolean := false;  -- implement data cache
       DCACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;                 -- d-cache: number of blocks (min 1), has to be a power of 2
       DCACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;                -- d-cache: block size in bytes (min 4), has to be a power of 2

		 -- External bus interface (XBUS) --
		 XBUS_EN                    	: boolean                        := false;       -- implement external memory bus interface?
		 XBUS_TIMEOUT               	: natural                        := 255;         -- cycles after a pending bus access auto-terminates (0 = disabled)
		 XBUS_REGSTAGE_EN           	: boolean                        := false;       -- add XBUS register stage
		 XBUS_CACHE_EN              	: boolean                        := false;       -- enable external bus cache (x-cache)
		 XBUS_CACHE_NUM_BLOCKS      	: natural range 1 to 256         := 64;          -- x-cache: number of blocks (min 1), has to be a power of 2
		 XBUS_CACHE_BLOCK_SIZE      	: natural range 1 to 2**16       := 32;          -- x-cache: block size in bytes (min 4), has to be a power of 2

		 -- Execute in-place module (XIP) --
		 XIP_EN                     	: boolean                        := false;       -- implement execute in place module (XIP)?
		 XIP_CACHE_EN               	: boolean                        := false;       -- implement XIP cache?
		 XIP_CACHE_NUM_BLOCKS       	: natural range 1 to 256         := 8;           -- number of blocks (min 1), has to be a power of 2
		 XIP_CACHE_BLOCK_SIZE       	: natural range 1 to 2**16       := 256;         -- block size in bytes (min 4), has to be a power of 2
       
		 -- Processor peripherals --
		 IO_DISABLE_SYSINFO         	: boolean                        := false;       -- disable the SYSINFO module (for advanced users only)
       IO_GPIO_NUM                  : natural range 0 to 64          := 0;           -- number of GPIO input/output pairs (0..64)
       IO_MTIME_EN                  : boolean := false;  -- implement machine system timer (MTIME)?
       IO_UART0_EN                  : boolean := false;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
       IO_UART0_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_UART0_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_UART1_EN                  : boolean := false;  -- implement secondary universal asynchronous receiver/transmitter (UART1)?
       IO_UART1_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_UART1_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_SPI_EN                    : boolean := false;  -- implement serial peripheral interface (SPI)?
       IO_SPI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be a power of two, min 1
       IO_SDI_EN                    : boolean := false;  -- implement serial data interface (SDI)?
       IO_SDI_FIFO                  : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
       IO_TWI_EN                    : boolean := false;  -- implement two-wire interface (TWI)?
		 IO_TWI_FIFO                : natural range 1 to 2**15       := 1;           -- RTX fifo depth, has to be zero or a power of two, min 1
		 IO_PWM_NUM_CH                : natural range 0 to 12          := 0;           -- number of PWM channels to implement (0..12); 0 = disabled
       IO_WDT_EN                    : boolean := false;  -- implement watch dog timer (WDT)?
       IO_TRNG_EN                   : boolean := false;  -- implement true random number generator (TRNG)?
       IO_TRNG_FIFO                 : natural range 1 to 2**15       := 1;           -- data fifo depth, has to be a power of two, min 1
       IO_CFS_EN                    : boolean := false;  -- implement custom functions subsystem (CFS)?
       IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000"; -- custom CFS configuration generic
       IO_CFS_IN_SIZE               : natural := 32;     -- size of CFS input conduit in bits
       IO_CFS_OUT_SIZE              : natural := 32;     -- size of CFS output conduit in bits
       IO_NEOLED_EN                 : boolean := false;  -- implement NeoPixel-compatible smart LED interface (NEOLED)?
       IO_NEOLED_TX_FIFO            : natural range 1 to 2**15       := 1;           -- NEOLED FIFO depth, has to be a power of two, min 1
       IO_GPTMR_EN                  : boolean := false;  -- implement general purpose timer (GPTMR)?
       IO_ONEWIRE_EN                : boolean                        := false;       -- implement 1-wire interface (ONEWIRE)?
       IO_DMA_EN                    : boolean                        := false;       -- implement direct memory access controller (DMA)?
       IO_SLINK_EN                  : boolean                        := false;       -- implement stream link interface (SLINK)?
       IO_SLINK_RX_FIFO             : natural range 1 to 2**15       := 1;           -- RX fifo depth, has to be a power of two, min 1
       IO_SLINK_TX_FIFO             : natural range 1 to 2**15       := 1;           -- TX fifo depth, has to be a power of two, min 1
       IO_CRC_EN                    : boolean                        := false        -- implement cyclic redundancy check unit (CRC)?
     );
     port (
	  
       -- Global control --
       clk_i          : in  std_ulogic; -- global clock, rising edge
       rstn_i         : in  std_ulogic; -- global reset, low-active, async

		 -- External bus interface (available if XBUS_EN = true) --
		 xbus_adr_o     : out std_ulogic_vector(31 downto 0);                    -- address
		 xbus_dat_o     : out std_ulogic_vector(31 downto 0);                    -- write data
		 xbus_tag_o     : out std_ulogic_vector(2 downto 0);                     -- access tag
		 xbus_we_o      : out std_ulogic;                                        -- read/write
		 xbus_sel_o     : out std_ulogic_vector(3 downto 0);                     -- byte enable
		 xbus_stb_o     : out std_ulogic;                                        -- strobe
		 xbus_cyc_o     : out std_ulogic;                                        -- valid cycle
		 xbus_dat_i     : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- read data
		 xbus_ack_i     : in  std_ulogic := 'L';                                 -- transfer acknowledge
		 xbus_err_i     : in  std_ulogic := 'L';                                 -- transfer error

		 -- Stream Link Interface (available if IO_SLINK_EN = true) --
		 slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'L'); -- RX input data
		 slink_rx_src_i : in  std_ulogic_vector(3 downto 0)  := (others => 'L'); -- RX source routing information
		 slink_rx_val_i : in  std_ulogic := 'L';                                 -- RX valid input
		 slink_rx_lst_i : in  std_ulogic := 'L';                                 -- RX last element of stream
		 slink_rx_rdy_o : out std_ulogic;                                        -- RX ready to receive
		 slink_tx_dat_o : out std_ulogic_vector(31 downto 0);                    -- TX output data
		 slink_tx_dst_o : out std_ulogic_vector(3 downto 0);                     -- TX destination routing information
		 slink_tx_val_o : out std_ulogic;                                        -- TX valid output
		 slink_tx_lst_o : out std_ulogic;                                        -- TX last element of stream
		 slink_tx_rdy_i : in  std_ulogic := 'L';                                 -- TX ready to send

       -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
       xip_csn_o      : out std_ulogic; -- chip-select, low-active
       xip_clk_o      : out std_ulogic; -- serial clock
       xip_dat_i      : in  std_ulogic := 'L'; -- device data input
       xip_dat_o      : out std_ulogic; -- controller data output

       -- GPIO (available if IO_GPIO_NUM > 0) --
       gpio_o         : out std_ulogic_vector(63 downto 0); -- parallel output
       gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U'); -- parallel input

       -- primary UART0 (available if IO_UART0_EN = true) --
       uart0_txd_o    : out std_ulogic; -- UART0 send data
       uart0_rxd_i    : in  std_ulogic := 'L'; -- UART0 receive data
       uart0_rts_o    : out std_ulogic; -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
       uart0_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART0.TX allowed to transmit, low-active, optional

       -- secondary UART1 (available if IO_UART1_EN = true) --
       uart1_txd_o    : out std_ulogic; -- UART1 send data
       uart1_rxd_i    : in  std_ulogic := 'L'; -- UART1 receive data
       uart1_rts_o    : out std_ulogic; -- HW flow control: UART1.RX ready to receive ("RTR"), low-active, optional
       uart1_cts_i    : in  std_ulogic := 'L'; -- HW flow control: UART1.TX allowed to transmit, low-active, optional

       -- SPI (available if IO_SPI_EN = true) --
       spi_clk_o      : out std_ulogic; -- SPI serial clock
       spi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
       spi_dat_i      : in  std_ulogic := 'L'; -- controller data in, peripheral data out
       spi_csn_o      : out std_ulogic_vector(07 downto 0); -- chip-select

       -- SDI (available if IO_SDI_EN = true) --
       sdi_clk_i      : in  std_ulogic := 'L'; -- SDI serial clock
       sdi_dat_o      : out std_ulogic; -- controller data out, peripheral data in
       sdi_dat_i      : in  std_ulogic := 'L'; -- controller data in, peripheral data out
       sdi_csn_i      : in  std_ulogic := 'H'; -- chip-select

       -- TWI (available if IO_TWI_EN = true) --
       twi_sda_i      : in  std_ulogic := 'H'; -- serial data line sense input
       twi_sda_o      : out std_ulogic; -- serial data line output (pull low only)
       twi_scl_i      : in  std_ulogic := 'H'; -- serial clock line sense input
       twi_scl_o      : out std_ulogic; -- serial clock line output (pull low only)

       -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
       onewire_i      : in  std_ulogic := 'H'; -- 1-wire bus sense input
       onewire_o      : out std_ulogic; -- 1-wire bus output (pull low only)

       -- PWM (available if IO_PWM_NUM_CH > 0) --
       pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels

       -- Custom Functions Subsystem IO (available if IO_CFS_EN = true) --
       cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'L'); -- custom CFS inputs conduit
       cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0); -- custom CFS outputs conduit

       -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
       neoled_o       : out std_ulogic; -- async serial data line
		 
		 -- Machine timer system time (available if IO_MTIME_EN = true) --
		 mtime_time_o   : out std_ulogic_vector(63 downto 0);                    -- current system time

		 
		 counter        : in std_logic_vector(31 downto 0);
       sector         : in std_logic_vector(2 downto 0)
     );
   end component neorv32_top;


   --------------------------------------------------------
   -- Define all local signals here
   --------------------------------------------------------

   signal sys_clk          : std_logic := '0';
   signal pll_locked       : std_logic := '0';
   signal reset            : std_logic := '0';
   signal reset_s1         : std_logic := '1';
   signal reset_s2         : std_logic := '1';
   signal reset_s3         : std_logic := '1';
   signal sys_rst          : std_logic;
   signal fpga_reset       : std_logic;

   signal clk_i            : std_logic;
   signal rstn_i           : std_logic;
	
	-- signal gpio             : std_ulogic_vector(63 downto 0);
	signal gpio_o_signal : std_ulogic_vector(31 downto 0);
	signal gpio_i_signal : std_ulogic_vector(31 downto 0);
	
	signal PWM_u : std_ulogic_vector(3 downto 0);
	
	signal signal_couter : std_logic_vector(31 downto 0);

   signal signal_sector           : std_logic_vector(2 downto 0);
 

begin

   --
   -- Edge Counter
   --
   inst_counter : counter
      port map (
         rst => reset,
         clk => sys_clk,
         signal_in => HALL_i,
         counter => signal_couter
      );


   --
   -- HALL Sector
   --
   inst_hall_sector : hall_sector
      port map (
         rst => reset,
         clk => sys_clk,
         signal_in => HALL_SENSOR,
         sector => signal_sector
      );

   --
   -- PLL
   --
   inst_pll_sys : pll_sys
      port map (
         inclk0 => CLOCK_50,
         c0     => sys_clk,
         c1     => open,        
         locked => pll_locked
      );

   --
   -- In general it is a bad idea to use an asynchhronous Reset signal.
   -- But it is only a bad idea in case of asynchhronous deasserting.
   -- Therefore the deasserting of the Reset signal must be synchronized.
   --

   -- Asynchronous assert
   fpga_reset <= '1' when (FPGA_RESET_N = '0') else '0';
   reset      <= '1' when ((fpga_reset = '1') OR (pll_locked = '0')) else '0';

   -- Synchronize deassert
   process (sys_clk, reset)
   begin
      if (reset = '1') then
         reset_s1 <= '1';
         reset_s2 <= '1';
         reset_s3 <= '1';
      elsif rising_edge(sys_clk) then
         reset_s1 <= '0';
         reset_s2 <= reset_s1;
         reset_s3 <= reset_s2;
      end if;
   end process;

   -- The deassert edge is now synchronized
   sys_rst <= reset_s3;
   clk_i  <= sys_clk;
   rstn_i <= not sys_rst;

   --
   -- neorv32
   --
   neorv32_top_inst: neorv32_top
      generic map (
         -- General --
         CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
         INT_BOOTLOADER_EN            => true,             -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM

         -- On-Chip Debugger (OCD) --
         ON_CHIP_DEBUGGER_EN          => false,              -- implement on-chip debugger

         -- RISC-V CPU Extensions --
         CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
         CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
			CPU_EXTENSION_RISCV_Zfinx    => true,              -- implement 32-bit floating-point extension (using INT regs!)
         CPU_EXTENSION_RISCV_Zicntr   => true,              -- implement base counters?
	
         -- Internal Instruction memory --
         MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
         MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes

         -- Internal Data memory --
         MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
         MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes

         -- Processor peripherals --
         IO_GPIO_NUM                  => 32,                 -- number of GPIO input/output pairs (0..64)
         IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
         IO_UART0_EN                  => true,               -- implement primary universal asynchronous receiver/transmitter (UART0)?
			IO_PWM_NUM_CH					  => 4,						-- number of PWM channels to implement (0..12); 0 = disabled
			
			IO_GPTMR_EN                  => true              -- implement general purpose timer (GPTMR)?
		)
      port map (
         -- Global control --
         clk_i         => clk_i,                            -- global clock, rising edge
         rstn_i        => rstn_i,                           -- global reset, low-active, async

         -- GPIO (available if IO_GPIO_EN = true) --
         gpio_o(31 downto 0) => gpio_o_signal(31 downto 0),                     -- parallel output
         gpio_i(31 downto 0) => gpio_i_signal(31 downto 0),							-- parallel input                            -- parallel input

         -- primary UART0 (available if IO_UART0_EN = true) --
         uart0_txd_o   => UART0_TXD,                        -- UART0 send data
         uart0_rxd_i   => UART0_RXD,                         -- UART0 receive data
			pwm_o(3 downto 0)         => PWM_u,                   -- pwm channels
			
			counter => signal_couter,
         sector => signal_sector
		);
   --------------------------------------------------------
   -- Output/Input signals
   --------------------------------------------------------
   -- LED <= ADC_OUT(7 downto 0);
   LED    <= To_StdLogicVector( gpio_o_signal(7 downto 0) ); -- The 
	
	gpio_i_signal(15 downto 0) <=  to_stdulogicvector( "000" & GPIO_i );  -- Atribuindo os bits de GPIO_i
	GPIO_o <= To_StdLogicVector( gpio_o_signal(12 downto 0) );
	
	PWM <= std_logic_vector(PWM_u);
	sys_clk_port <= sys_clk;
	
end architecture syn;

-- *** EOF ***
