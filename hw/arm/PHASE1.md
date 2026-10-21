# Fase 1 — base de boot do QSD8x50 (Nexus One)

**Status: escrito, mas NÃO compilado nem executado.** Foi feito contra a API do
seu fork (conferi `pl011.c`, `pl011.h`, `meson.build`, `Kconfig`, `nexus-one.c`),
mas ninguém rodou `ninja` ainda. Espere corrigir alguns erros de compilação.

## O que foi adicionado

| Arquivo | O que faz |
|---|---|
| `include/hw/arm/qsd8x50.h` | Endereços, números de IRQ e nomes de tipos |
| `hw/arm/qsd8x50_vic.c` | Controlador de interrupções (64 linhas, IRQ/FIQ, edge/level) |
| `hw/arm/qsd8x50_timer.c` | Timers GPT (32768 Hz) e DGT (19,2 MHz / divisor) |
| `hw/arm/qsd8x50_uart.c` | UART não-DM para console (TX/RX, ISR/IMR, comandos CR) |
| `hw/arm/nexus-one.c` | Liga tudo + região "pega-tudo" que loga acessos não mapeados |

## Aplicar

```sh
./apply.sh /caminho/do/seu/qemu
cd /caminho/do/seu/qemu/build && ninja qemu-system-arm
```

O script copia os arquivos e troca, no `hw/arm/meson.build`:
`files('nexus-one.c')` → `files('nexus-one.c', 'qsd8x50_vic.c', 'qsd8x50_timer.c', 'qsd8x50_uart.c')`.
Nenhuma mudança de Kconfig é necessária.

## Rodar

```sh
qemu-system-arm -M nexus-one -m 256 \
  -kernel zImage -initrd ramdisk.img \
  -append "console=ttyMSM0,115200n8 ..." \
  -nographic \
  -d unimp,guest_errors -D qemu.log \
  -global qsd8x50-vic.debug=on -global qsd8x50-uart.debug=on
```

- O `boot.img` original traz `console=null`; troque para o console da UART1
  (`ttyMSM0` é o nome mais provável nesse kernel; confira no `.config`).
- `-serial N` vai para UART(N+1). UART1 = serial 0 = console.
- `qemu.log` é a ferramenta principal: cada linha `unmapped read/write @0x...`
  é um periférico que o kernel tentou usar.

## ⚠️ Constantes NÃO verificadas

Escrevi de memória do kernel android-msm 2.6.29. Confira no **seu** kernel:

- `arch/arm/mach-msm/include/mach/msm_iomap-8x50.h` → bases do VIC (`0xAC000000`),
  CSR/timers (`0xAC100000`) e UARTs (`0xA9A00000/B/C`).
- `arch/arm/mach-msm/include/mach/irqs-8x50.h` → números de IRQ (timer GPT = 7,
  DGT = 8, UART1/2/3 = 9/10/11).
- `arch/arm/mach-msm/irq.c` e `entry-macro.S` → quais registradores do VIC o
  kernel realmente lê (implementei tanto `IRQ_STATUS` quanto `VEC_RD`).
- `arch/arm/mach-msm/timer.c` → offsets GPT/DGT e divisor do DGT.
- Qual UART é o console no `board-mahimahi.c`.

Se algo estiver errado, o `qemu.log` mostra acessos a endereços desconhecidos
logo no começo.

## Simplificações conscientes

- `POLARITY` do VIC é só armazenado; todas as linhas são tratadas como ativas em nível alto.
- `MASTEREN` do VIC reseta em 1 (no hardware seria 0).
- UART: TX síncrono, FIFO de TX sempre vazio, baud rate ignorado, sem DM.

## Próximas paredes previsíveis (em ordem)

1. **`proc_comm`** — o kernel pede clocks/regulators ao modem ARM9 pela memória
   compartilhada e fica em loop esperando resposta. Provável 1º travamento.
2. **SMD** (canal de memória compartilhada com o modem) — base para RIL/rádio.
3. **Clocks/PLLs e GPIO** do SoC.
4. **Core Scorpion**: o kernel pode acessar registradores CP15 específicos da
   Qualcomm (cache L2) que o Cortex-A8 do QEMU não tem.
5. NAND, MDDI/display, touchscreen, I2C, SD — nessa ordem de necessidade.

Áudio, microfone e câmera (ADSP/QDSP5) só fazem sentido depois do passo 1–3.
