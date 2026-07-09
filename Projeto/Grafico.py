import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import numpy as np
from collections import deque

# --- Configurações ---
BROKER = "10.0.105.207" # Ajuste para o seu IP
TOPICO = "esp32/sinal"  # Verifique se este é o tópico exato que a ESP32 publica
FATOR_CALIBRACAO = 0.005 # Ajuste conforme necessário

# --- Configuração dos buffers de Métricas ---
MAX_PONTOS = 50 
dados_rms_mv   = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)
dados_hz       = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)
dados_corrente = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)

# Variáveis globais para armazenar os dados do último JSON recebido
ultimas_harmonicas = []
ultimo_mean_raw = 1860.0  
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

# 3. RECONSTRUÇÃO DO SINAL (Soma de Fourier)
linha_sinal, = axs[1, 0].plot([], [], 'g-', linewidth=2)
axs[1, 0].set_title('Sinal Reconstruído no Tempo')
axs[1, 0].set_xlabel('Tempo (ms)')
axs[1, 0].set_ylabel('Amplitude (Raw)')
axs[1, 0].set_xlim(0, 50)  # Visualizando 50 milissegundos de onda 
axs[1, 0].set_ylim(0, 4095) # Limite ajustado para o range real do ADC (0 a 4095)

# 4. Corrente
linha_corrente, = axs[1, 1].plot([], [], 'm-', linewidth=2)
axs[1, 1].set_title('Corrente (A)')
axs[1, 1].set_xlim(0, MAX_PONTOS)
axs[1, 1].set_ylim(0, 5) # Ajuste o limite máximo de corrente aqui se necessário

# --- Função MQTT: Chamada sempre que chega mensagem ---
def on_message(client, userdata, msg):
    # Declaramos globais para poder alterá-las
    global ultimo_mean_raw, ultimas_harmonicas

    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        
        # 1. Atualiza as variáveis da onda
        ultimo_mean_raw = payload.get("mean_raw", 0.0)
        ultimas_harmonicas = payload.get("harmonicas", [])
        
        # 2. Extrai e guarda as métricas numéricas nos buffers do gráfico
        rms_mv = payload.get("rms_mv", 0.0)
        mains_hz = payload.get("mains_hz", 0.0)
        rms_raw = payload.get("rms_raw", 0.0)
        
        dados_rms_mv.append(rms_mv)
        dados_hz.append(mains_hz)
        
        # Cálculo da corrente real
        corrente = rms_raw * FATOR_CALIBRACAO
        dados_corrente.append(corrente)
        
        print(f"MQTT Recebido | V_RMS: {rms_mv}mV | Corrente: {corrente:.2f}A | Harmónicas: {len(ultimas_harmonicas)}")

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
    
    # Reconstrói a onda se houver dados de FFT
    if ultimas_harmonicas:
        # Cria um vetor de tempo de 0 a 0.05 segundos (50 ms)
        t = np.linspace(0, 0.05, 500) 
        
        # Inicia a onda já no nível de tensão média (Offset DC)
        onda_reconstruida = np.ones_like(t) * ultimo_mean_raw
        
        # Soma as harmónicas
        for h in ultimas_harmonicas:
            freq = h.get("f", 0)
            amp_bruta = h.get("a", 0)
            fase = h.get("p", 0)
            
            if freq > 0:
                # Normalização padrão da FFT para converter Magnitude de volta em Amplitude ADC
                amp_real = (2.0 * amp_bruta) / TAMANHO_FFT
                
                # Equação da onda: A * cos(2*pi*f*t + fase)
                onda_reconstruida += amp_real * np.cos(2 * np.pi * freq * t + fase)
            
        # O eixo X do gráfico mostrará o tempo em milissegundos (t * 1000)
        linha_sinal.set_data(t * 1000, onda_reconstruida)
        
        # Opcional: auto-ajuste do eixo Y da onda para não ficar sempre estático em 0-4095
        margem = 300
        min_y = max(0, min(onda_reconstruida) - margem)
        max_y = min(4095, max(onda_reconstruida) + margem)
        axs[1, 0].set_ylim(min_y, max_y)
        
    return linha_rms_mv, linha_hz, linha_sinal, linha_corrente

# --- Fechamento limpo ---
def ao_fechar(event):
    print("A encerrar a ligação MQTT e a fechar os gráficos...")
    client.loop_stop()
    client.disconnect()

fig.canvas.mpl_connect('close_event', ao_fechar)

# Inicia a animação (blit=False para evitar bugs de eixos não atualizados em alguns sistemas)
ani = FuncAnimation(fig, atualizar_grafico, blit=False, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()