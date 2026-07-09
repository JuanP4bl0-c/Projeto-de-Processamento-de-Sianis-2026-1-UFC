# Projeto Leitura de Corrente

Este firmware foi reorganizado para monitorar corrente alternada com o sensor ZMCT103C e o amplificador LM358, lendo o sinal pelo ADC do ESP32, removendo o offset DC e aplicando FFT para destacar os componentes de frequência presentes no consumo dos eletrodomésticos.

## Fluxo atual

1. O ADC é configurado no GPIO36, via ADC1, usando o canal ADC0.
2. Uma task dedicada coleta uma janela de amostras em taxa fixa.
3. O sinal passa por remoção de média e janela de Hann.
4. A FFT é executada e os picos espectrais são impressos no terminal.
5. O log mostra o valor bruto inicial, a tensão estimada e um preview das primeiras amostras da janela.

## Ajustes rápidos

Se o circuito estiver ligado em outro pino ADC1, altere `CURRENT_SENSOR_GPIO` e `CURRENT_SENSOR_ADC_CHANNEL` em `main/current_monitor.c`.

Se quiser mais resolução em frequência, aumente `FFT_SIZE`.

Se quiser mais faixa temporal, altere `SAMPLE_RATE_HZ` ou o atraso entre janelas.
