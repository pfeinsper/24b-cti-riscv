library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity hall_sector is
    Port (
        clk : in std_logic;
        rst : in std_logic;
        signal_in : in std_logic_vector(2 downto 0);
        sector : out std_logic_vector(2 downto 0)
    );
end hall_sector;

architecture Behavioral of hall_sector is

    signal prev_signal : std_logic_vector(2 downto 0) := "111";
    signal stable_signal : std_logic_vector(2 downto 0) := "111";
    signal debounce_counter : integer := 0;
    constant debounce_limit : integer := 1000;

begin

    process(clk, rst)
    begin
			if rst = '1' then
            debounce_counter <= 0;
            stable_signal <= "111";
			else
			
				if signal_in /= prev_signal then
						 debounce_counter <= 0;
				else
					if debounce_counter < debounce_limit then
                    debounce_counter <= debounce_counter + 1;
               else
                    stable_signal <= signal_in;
               end if;
            end if;
				
            prev_signal <= signal_in;
        end if;
    end process;

    with stable_signal select
        sector <= "100" when "001", -- 001 to 4
                  "010" when "010", -- 010 to 2
                  "011" when "011", -- 011 to 3
                  "000" when "100", -- 100 to 0
                  "101" when "101", -- 101 to 5
                  "001" when "110", -- 110 to 1
                  -- others undefined
                  "111" when others; -- 111 to 7

end Behavioral;