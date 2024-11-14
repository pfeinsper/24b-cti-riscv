library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
--use IEEE.STD_LOGIC_ARITH.ALL;
--use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity counter is
    Port (
       -- clk : in std_logic;         -- Clock do sistema
       -- rst : in std_logic;         -- Sinal de reset
       -- signal_in : in std_logic;   -- Sinal de entrada digital (com bouncing)
        signal_in : in std_logic_vector(2 downto 0);
        sector : out std_logic_vector(2 downto 0)  -- Valor do contador
    );
end counter;

architecture Behavioral of counter is

begin

    with signal_in select
        counter <="100" when "001", -- 001 to 4
                  "010" when "010", -- 010 to 2
                  "011" when "011", -- 011 to 3
                  "000" when "100", -- 100 to 0
                  "101" when "101", -- 101 to 5
                  "001" when "110", -- 110 to 1
                  -- others undefined
                  others => "111"; -- 111 to 7
end Behavioral;