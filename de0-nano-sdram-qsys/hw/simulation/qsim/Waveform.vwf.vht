-- Copyright (C) 2020  Intel Corporation. All rights reserved.
-- Your use of Intel Corporation's design tools, logic functions 
-- and other software and tools, and any partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Intel Program License 
-- Subscription Agreement, the Intel Quartus Prime License Agreement,
-- the Intel FPGA IP License Agreement, or other applicable license
-- agreement, including, without limitation, that your use is for
-- the sole purpose of programming logic devices manufactured by
-- Intel and sold by Intel or its authorized distributors.  Please
-- refer to the applicable agreement for further details, at
-- https://fpgasoftware.intel.com/eula.

-- *****************************************************************************
-- This file contains a Vhdl test bench with test vectors .The test vectors     
-- are exported from a vector file in the Quartus Waveform Editor and apply to  
-- the top level entity of the current Quartus project .The user can use this   
-- testbench to simulate his design using a third-party simulation tool .       
-- *****************************************************************************
-- Generated on "10/28/2024 15:25:21"
                                                             
-- Vhdl Test Bench(with test vectors) for design  :          top
-- 
-- Simulation tool : 3rd Party
-- 

LIBRARY ieee;                                               
USE ieee.std_logic_1164.all;                                

ENTITY top_vhd_vec_tst IS
END top_vhd_vec_tst;
ARCHITECTURE top_arch OF top_vhd_vec_tst IS
-- constants                                                 
-- signals                                                   
SIGNAL ADC_CS_N : STD_LOGIC;
SIGNAL ADC_SADDR : STD_LOGIC;
SIGNAL ADC_SCLK : STD_LOGIC;
SIGNAL ADC_SDAT : STD_LOGIC;
SIGNAL CLOCK_50 : STD_LOGIC;
SIGNAL counter : STD_LOGIC_VECTOR(11 DOWNTO 0);
SIGNAL GPIO_2 : STD_LOGIC_VECTOR(12 DOWNTO 0);
SIGNAL GPIO_2_IN : STD_LOGIC_VECTOR(2 DOWNTO 0);
SIGNAL HALL_GPIO_i : STD_LOGIC;
SIGNAL KEY : STD_LOGIC_VECTOR(1 DOWNTO 0);
SIGNAL LED : STD_LOGIC_VECTOR(7 DOWNTO 0);
SIGNAL memory_counter : STD_LOGIC_VECTOR(31 DOWNTO 0);
SIGNAL MSW_IRQ : STD_LOGIC;
SIGNAL SW : STD_LOGIC_VECTOR(3 DOWNTO 0);
SIGNAL sys_clk_port : STD_LOGIC;
SIGNAL TCK_i : STD_LOGIC;
SIGNAL TDI_i : STD_LOGIC;
SIGNAL TDO_o : STD_LOGIC;
SIGNAL TMS_i : STD_LOGIC;
SIGNAL UART0_RXD : STD_LOGIC;
SIGNAL UART0_TXD : STD_LOGIC;
SIGNAL XIRQ : STD_LOGIC_VECTOR(31 DOWNTO 0);
COMPONENT top
	PORT (
	ADC_CS_N : OUT STD_LOGIC;
	ADC_SADDR : OUT STD_LOGIC;
	ADC_SCLK : OUT STD_LOGIC;
	ADC_SDAT : IN STD_LOGIC;
	CLOCK_50 : IN STD_LOGIC;
	counter : OUT STD_LOGIC_VECTOR(11 DOWNTO 0);
	GPIO_2 : OUT STD_LOGIC_VECTOR(12 DOWNTO 0);
	GPIO_2_IN : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
	HALL_GPIO_i : IN STD_LOGIC;
	KEY : IN STD_LOGIC_VECTOR(1 DOWNTO 0);
	LED : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
	memory_counter : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
	MSW_IRQ : IN STD_LOGIC;
	SW : IN STD_LOGIC_VECTOR(3 DOWNTO 0);
	sys_clk_port : OUT STD_LOGIC;
	TCK_i : IN STD_LOGIC;
	TDI_i : IN STD_LOGIC;
	TDO_o : OUT STD_LOGIC;
	TMS_i : IN STD_LOGIC;
	UART0_RXD : IN STD_LOGIC;
	UART0_TXD : OUT STD_LOGIC;
	XIRQ : IN STD_LOGIC_VECTOR(31 DOWNTO 0)
	);
END COMPONENT;
BEGIN
	i1 : top
	PORT MAP (
-- list connections between master ports and signals
	ADC_CS_N => ADC_CS_N,
	ADC_SADDR => ADC_SADDR,
	ADC_SCLK => ADC_SCLK,
	ADC_SDAT => ADC_SDAT,
	CLOCK_50 => CLOCK_50,
	counter => counter,
	GPIO_2 => GPIO_2,
	GPIO_2_IN => GPIO_2_IN,
	HALL_GPIO_i => HALL_GPIO_i,
	KEY => KEY,
	LED => LED,
	memory_counter => memory_counter,
	MSW_IRQ => MSW_IRQ,
	SW => SW,
	sys_clk_port => sys_clk_port,
	TCK_i => TCK_i,
	TDI_i => TDI_i,
	TDO_o => TDO_o,
	TMS_i => TMS_i,
	UART0_RXD => UART0_RXD,
	UART0_TXD => UART0_TXD,
	XIRQ => XIRQ
	);

-- CLOCK_50
t_prcs_CLOCK_50: PROCESS
BEGIN
LOOP
	CLOCK_50 <= '0';
	WAIT FOR 10000 ps;
	CLOCK_50 <= '1';
	WAIT FOR 10000 ps;
	IF (NOW >= 1000000 ps) THEN WAIT; END IF;
END LOOP;
END PROCESS t_prcs_CLOCK_50;

-- HALL_GPIO_i
t_prcs_HALL_GPIO_i: PROCESS
BEGIN
	FOR i IN 1 TO 8
	LOOP
		HALL_GPIO_i <= '0';
		WAIT FOR 60000 ps;
		HALL_GPIO_i <= '1';
		WAIT FOR 60000 ps;
	END LOOP;
	HALL_GPIO_i <= '0';
WAIT;
END PROCESS t_prcs_HALL_GPIO_i;

-- KEY[0]
t_prcs_KEY_0: PROCESS
BEGIN
	KEY(0) <= '1';
WAIT;
END PROCESS t_prcs_KEY_0;
END top_arch;
