library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;
context vunit_lib.vc_context;

use work.uart_rx_pkg.all;

entity uart_rx is
  generic (handle : uart_rx_t);
  port (
    clk : in std_ulogic;
    uart_txd : in std_ulogic
    );
end entity;

architecture a of uart_rx is
  signal uart_rx_sync : std_ulogic_vector(04 downto 0) := (others => '1');
  signal uart_rx_busy : std_ulogic := '0';
  signal uart_rx_sreg : std_ulogic_vector(08 downto 0) := (others => '0');
  signal uart_rx_baud_cnt : real;
  signal uart_rx_bitcnt : natural;

  file file_uart_tx_out : text open write_mode is "neorv32.testbench_" & get_name(handle.p_logger) & ".out";
  constant checker : checker_t := new_checker(handle.p_logger);
  constant character_queue : queue_t := new_queue;

begin
  control : process
    variable request_msg, reply_msg : msg_t;
    variable msg_type : msg_type_t;

    procedure put_characters_in_queue(s : string) is
    begin
      for idx in s'range loop
        push(character_queue, s(idx));
      end loop;
    end procedure put_characters_in_queue;
  begin
    receive(net, handle.p_actor, request_msg);
    msg_type := message_type(request_msg);

    -- Standard handling of standard wait_for_time messages = wait for the given time
    -- before proceeeding
    handle_wait_for_time(net, msg_type, request_msg);

    if msg_type = check_uart_msg then
      put_characters_in_queue(pop(request_msg));

    -- Custom handling of standard wait_until_idle message
    elsif msg_type = wait_until_idle_msg then
      while not is_empty(character_queue) loop
        wait until rising_edge(clk);
      end loop;
      reply_msg := new_msg(wait_until_idle_reply_msg);
      reply(net, request_msg, reply_msg);

    else
      unexpected_msg_type(msg_type);
    end if;
  end process;

  uart_rx_console : process(clk)
    variable i : integer;
    variable l : line;
    variable expected_character : character;
  begin
    -- "UART" --
    if rising_edge(clk) then
      -- synchronizer --
      uart_rx_sync <= uart_rx_sync(3 downto 0) & uart_txd;
      -- arbiter --
      if (uart_rx_busy = '0') then  -- idle
        uart_rx_busy <= '0';
        uart_rx_baud_cnt <= round(0.5 * handle.p_baud_val);
        uart_rx_bitcnt <= 9;
        if (uart_rx_sync(4 downto 1) = "1100") then  -- start bit? (falling edge)
          uart_rx_busy <= '1';
        end if;
      else
        if (uart_rx_baud_cnt <= 0.0) then
          if (uart_rx_bitcnt = 1) then
            uart_rx_baud_cnt <= round(0.5 * handle.p_baud_val);
          else
            uart_rx_baud_cnt <= round(handle.p_baud_val);
          end if;
          if (uart_rx_bitcnt = 0) then
            uart_rx_busy <= '0';  -- done
            i := to_integer(unsigned(uart_rx_sreg(8 downto 1)));

            if is_empty(character_queue) then
              check_failed(checker, "Extra characters received");
            else
              expected_character := pop(character_queue);
              check_equal(checker, character'val(i), expected_character);
            end if;

            if (i = 10) then  -- Linux line break
              writeline(file_uart_tx_out, l);
            elsif (i /= 13) then  -- Remove additional carriage return
              write(l, character'val(i));
            end if;
          else
            uart_rx_sreg <= uart_rx_sync(4) & uart_rx_sreg(8 downto 1);
            uart_rx_bitcnt <= uart_rx_bitcnt - 1;
          end if;
        else
          uart_rx_baud_cnt <= uart_rx_baud_cnt - 1.0;
        end if;
      end if;
    end if;
  end process uart_rx_console;
end architecture;
