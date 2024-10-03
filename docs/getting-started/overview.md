# Project Structure

Ok, vamos escrever isso em português por enquanto, por valorização da agiidade.

Este projeto é organizado em três pastas principais (ignore as outras por enquanto):

- .devcontainer: Configurações do Visual Studio Code para o ambiente de desenvolvimento.
- acionamento_brushless: Código fonte do firmware do acionamento brushless.
- de0-nano-sdeam-qsys: Projeto do Quartus Prime para o FPGA DE0-Nano-SoC e também o código fonte do firmware do FPGA.	

## .devcontainer

Se você é do time de desenvolvemento e quer ser capaz de compilar os códigos C para o RISC-V esse é o seu principal companheiro. Aqui você encontra as configurações do Visual Studio Code para o ambiente de desenvolvimento.

Assim que você abrir o projeto no Visual Studio Code, ele vai te perguntar se você quer abrir o projeto no container. Aceite e ele vai montar um ambiente de desenvolvimento com todas as ferramentas necessárias para compilar o código C para o RISC-V.

O devcontainer contém a toolchain do RISC-V e o Quartus. Então, se você quiser compilar o código C para o RISC-V ou programar o FPGA, você deve fazer isso dentro do container.

Para abrir o Quartus, você pode abrir o terminal do Visual Studio Code e digitar `quartus` para abrir o Quartus.

## acionamento_brushless

Eu não sei muito dessa parte, vamos ageitar isso depois.

## de0-nano-sdeam-qsys

Esta pasta possui outras duas subpastas:

- hw: Projeto do Quartus Prime para o FPGA DE0-Nano-SoC.
- sw: Código fonte do firmware do FPGA.

Os arquivos da pasta hw que são mais importantes para o desenvolvimento são:

- de0-nano-soc.qpf: Arquivo de projeto do Quartus Prime. (use este arquivo para abrir o projeto no Quartus Prime)
- de0-nano-soc.qsf: Arquivo de configuração do projeto. (é onde estão as configurações de pinagem, use este arquivo sempre que precisar fazer uma montagem de pinos)
- src/top.vhd: Arquivo de descrição do hardware do FPGA. (no inicio e no final do arquivo estão configurados os sinais mais importantes, então é onde a sua dúvida provavelmente será respondida).

Dentro da pasta sw, vcoê encontrará mais duas pastas principais (ignore as outras por enquanto):

- example: São os códigos C do projeto (se baseie no golden_top para entender como o código C se comunica com o hardware).
- lib: Bibliotecas de suporte para o projeto.
