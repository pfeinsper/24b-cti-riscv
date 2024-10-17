library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.ALL;
use ieee.std_logic_unsigned.ALL;


entity edge_counter is
    Port (
        clk : in std_logic;         -- Clock do sistema
        rst : in std_logic;         -- Sinal de reset
        signal_in : in std_logic;   -- Sinal de entrada digital
        counter_out : out std_logic_vector(7 downto 0)  -- Valor do contador
    );
end edge_counter;

architecture Behavioral of edge_counter is
    signal prev_signal : std_logic := '0';   -- Sinal anterior para detectar borda
    signal counter : std_logic_vector(7 downto 0) := (others => '0');  -- Registrador do contador
begin

    process(clk, rst)
    begin
        if rst = '1' then
            -- Resetar o contador e sinal anterior
            counter <= (others => '0');
            prev_signal <= '0';

        elsif rising_edge(clk) then
            -- Detectar borda de descida (1 -> 0)
            if prev_signal = '1' and signal_in = '0' then
                counter <= counter + 1;
            end if;
            
            -- Atualizar o valor do sinal anterior
            prev_signal <= signal_in;
        end if;
    end process;

    -- Conectar o valor do contador à saída
    counter_out <= counter;

end Behavioral;
