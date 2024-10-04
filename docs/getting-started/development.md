# útil para o desenvolvimento.

## Manuais relevantes:

- [Datasheet do NEORV32](https://stnolting.github.io/neorv32/)
- [Guia do NEORV32](https://stnolting.github.io/neorv32/ug/)
- [Manual De0-Nano](../util/DE0_Nano_User_Manual.pdf)

## Como abrir e compilar código C para o RISC-V

1. Abra o projeto no Visual Studio Code.
2. Aceite a proposta de abrir o projeto no container.
3. Abra o terminal do Visual Studio Code.
4. Vá para a pasta de exemplos dentro de sw e escolha um exemplo.
5. Compile o código C com o comando `make clean all exe`.

Estas etapas gerarão um arquivo binário que pode ser carregado no RISC-V.

## Como carregar o software no RISC-V

Para carregar o software no RISC-V, siga estes passos:

1. garanta que você tem um adaptador USB-Serial conectado ao FPGA (use o qsf na pasta de hw para giar a pinagem).
2. Garanta que o NEORV32 está na FPGA através do Quartus Prime. (lembre que vcoê pode abrir o Quartus Prime digitando `quartus` no terminal do Visual Studio Code).
3. Estabeleça a comunicação com o FPGA através do adaptador USB-Serial, seguindo os passos descritos no [Guia do NEORV32](https://stnolting.github.io/neorv32/ug/#_uploading_and_starting_of_a_binary_executable_image_via_uart) e faça o upload do arquivo binário gerado na etapa anterior.