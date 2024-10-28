library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity edge_counter is
    Port (
        clk : in std_logic;         -- Clock do sistema
        rst : in std_logic;         -- Sinal de reset
        signal_in : in std_logic;   -- Sinal de entrada digital (com bouncing)
        counter : out std_logic_vector(11 downto 0)  -- Valor do contador
    );
end edge_counter;

architecture Behavioral of edge_counter is
    signal prev_signal : std_logic := '0';   -- Sinal anterior para detectar borda
    signal prev_stable_signal : std_logic := '0'; -- Sinal estabilizado (depois do debouncing) anterior
    signal stable_signal : std_logic := '0'; -- Sinal estabilizado (depois do debouncing)
    signal internal_counter : std_logic_vector(11 downto 0) := (others => '0');  -- Registrador do contador
    signal debounce_counter : integer := 0;  -- Temporizador para debouncing
    constant debounce_limit : integer := 3;  -- Limite de ciclos de clock para estabilizar (ajustável)

begin

    -- Processo de debouncing
    process(clk, rst)
    begin
        if rst = '1' then
            -- Resetar temporizador e sinal estabilizado
            debounce_counter <= 0;
            stable_signal <= '0';

        elsif falling_edge(clk) then
            -- Verificar se o sinal mudou de valor
            if signal_in /= prev_signal then
                -- Resetar o contador de debouncing quando o sinal oscilar
                debounce_counter <= 0;
            else
                -- Incrementar o contador de debouncing
                if debounce_counter < debounce_limit then
                    debounce_counter <= debounce_counter + 1;
                else
                    -- Quando o temporizador atinge o limite, estabilizamos o sinal
                    stable_signal <= signal_in;
                end if;
            end if;

            -- Atualizar o valor do sinal anterior
            prev_signal <= signal_in;
        end if;

    end process;

    -- Processo para contar bordas de descida do sinal estabilizado
    process(clk, rst)
    begin
        if rst = '1' then
            -- Resetar o contador e o sinal anterior estabilizado
            internal_counter <= (others => '0');
        elsif falling_edge(clk) then
            -- Detectar borda de descida no sinal estabilizado
            if stable_signal = '0' and prev_stable_signal = '1' then
                internal_counter <= internal_counter + 1;
            end if;

            -- Atualizar o valor do sinal estabilizado anterior
            prev_stable_signal <= stable_signal;
        end if;
    end process;

    -- Conectar o valor do contador à saída
    counter <= internal_counter;

end Behavioral;
