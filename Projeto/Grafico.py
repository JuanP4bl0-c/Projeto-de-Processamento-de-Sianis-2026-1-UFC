import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import numpy as np
from collections import deque

# --- Configurações ---
BROKER = "192.168.0.6"  # Ajuste para o seu IP
# BROKER = "10.10.220.31"  # Ajuste para o seu IP
# BROKER = "10.0.105.207"  # Ajuste para o seu IP
TOPICO = "esp32/sinal"   # Verifique se este é o tópico exato que a ESP32 publica
FATOR_CALIBRACAO = 0.0001236  # Ajuste conforme necessário

# --- Configuração dos buffers de Métricas ---
MAX_PONTOS = 50
dados_rms_mv   = deque([0] * MAX_PONTOS, maxlen=MAX_PONTOS)
dados_hz       = deque([0] * MAX_PONTOS, maxlen=MAX_PONTOS)
dados_corrente = deque([0] * MAX_PONTOS, maxlen=MAX_PONTOS)

# Variáveis globais para armazenar os dados do último JSON recebido
ultimas_harmonicas = []
TAMANHO_FFT = 1024

# --- Layout dos Gráficos ---
fig, axs = plt.subplots(2, 2, figsize=(12, 7))
fig.suptitle('Monitor de Energia e Harmónicas - ESP32', fontsize=14, fontweight='bold')

# 1. Tensão RMS
linha_rms_mv, = axs[0, 0].plot([], [], 'r-', linewidth=2)
axs[0, 0].set_title('Tensão RMS (mV)')
axs[0, 0].set_xlim(0, MAX_PONTOS)
axs[0, 0].set_ylim(0, 1000)

# 2. Frequência
linha_hz, = axs[0, 1].plot([], [], 'b-', linewidth=2)
axs[0, 1].set_title('Frequência (Hz)')
axs[0, 1].set_xlim(0, MAX_PONTOS)
axs[0, 1].set_ylim(55, 70)

# 3. ESPECTRO (harmónicas: frequência x amplitude)
axs[1, 0].set_title('Espectro (Harmónicas)')
axs[1, 0].set_xlabel('Frequência (Hz)')
axs[1, 0].set_ylabel('Amplitude (ADC)')
axs[1, 0].set_xlim(0, 500)
axs[1, 0].set_ylim(0, 100)

# 4. Corrente
linha_corrente, = axs[1, 1].plot([], [], 'm-', linewidth=2)
axs[1, 1].set_title('Corrente (A)')
axs[1, 1].set_xlim(0, MAX_PONTOS)
axs[1, 1].set_ylim(0, 5)  # Ajuste o limite máximo de corrente aqui se necessário


# --- Função MQTT: Chamada sempre que chega mensagem ---
def on_message(client, userdata, msg):
    global ultimas_harmonicas

    try:
        payload = json.loads(msg.payload.decode('utf-8'))

        # Atualiza as harmónicas recebidas
        ultimas_harmonicas = payload.get("harmonicas", [])

        # Extrai as métricas numéricas
        rms_mv = payload.get("rms_mv", 0.0)
        mains_hz = payload.get("mains_hz", 0.0)
        rms_raw = payload.get("rms_raw", 0.0)

        dados_rms_mv.append(rms_mv)
        dados_hz.append(mains_hz)

        # Cálculo da corrente real
        corrente = rms_raw * FATOR_CALIBRACAO
        dados_corrente.append(corrente)

        # Imprime no terminal as grandezas obtidas via MQTT
        print(f"[MQTT] Vrms: {rms_mv:8.2f} mV | Corrente: {corrente:6.3f} A | "
              f"Freq. rede: {mains_hz:5.2f} Hz | Harmónicas: {len(ultimas_harmonicas)}")

    except Exception as e:
        print(f"Erro ao processar pacote MQTT: {e}")


# --- Conexão MQTT ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPICO)
client.loop_start()


# --- Função de Animação: Desenha os gráficos a cada X milissegundos ---
def atualizar_grafico(frame):
    # Atualiza as linhas dos gráficos simples
    linha_rms_mv.set_data(range(len(dados_rms_mv)), dados_rms_mv)
    linha_hz.set_data(range(len(dados_hz)), dados_hz)
    linha_corrente.set_data(range(len(dados_corrente)), dados_corrente)

    # Redesenha o espectro (stem plot precisa ser limpo e refeito a cada frame)
    axs[1, 0].cla()
    axs[1, 0].set_title('Espectro (Harmónicas)')
    axs[1, 0].set_xlabel('Frequência (Hz)')
    axs[1, 0].set_ylabel('Amplitude (ADC)')

    if ultimas_harmonicas:
        freqs = []
        amps = []
        for h in ultimas_harmonicas:
            freq = h.get("f", 0)
            amp_bruta = h.get("a", 0)
            if freq > 0:
                # Normalização padrão da FFT: converte magnitude bruta em amplitude real
                amp_real = (2.0 * amp_bruta) / TAMANHO_FFT
                freqs.append(freq)
                amps.append(amp_real)

        if freqs:
            markerline, stemlines, baseline = axs[1, 0].stem(freqs, amps, basefmt=" ")
            plt.setp(stemlines, color='g', linewidth=2)
            plt.setp(markerline, color='g', markersize=6)

            axs[1, 0].set_xlim(0, max(freqs) * 1.2)
            axs[1, 0].set_ylim(0, max(amps) * 1.3 if max(amps) > 0 else 100)

    return linha_rms_mv, linha_hz, linha_corrente


# --- Fechamento limpo ---
def ao_fechar(event):
    print("A encerrar a ligação MQTT e a fechar os gráficos...")
    client.loop_stop()
    client.disconnect()


fig.canvas.mpl_connect('close_event', ao_fechar)

# Inicia a animação (blit=False, pois o espectro é redesenhado com cla() a cada frame)
ani = FuncAnimation(fig, atualizar_grafico, blit=False, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()