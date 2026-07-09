import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import numpy as np
from collections import deque

# --- Configurações ---
BROKER = "10.0.105.207"
TOPICO = "esp32/sinal"
FATOR_CALIBRACAO = 0.0001236

# --- Configuração dos buffers de Métricas ---
MAX_PONTOS = 50 
dados_rms_mv   = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)
dados_hz       = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)
dados_corrente = deque([0]*MAX_PONTOS, maxlen=MAX_PONTOS)

# Variáveis globais para armazenar os dados do último JSON recebido
ultimas_harmonicas = []
ultimo_mean_raw = 0.0  

# --- Layout dos Gráficos ---
fig, axs = plt.subplots(2, 2, figsize=(12, 7))
fig.suptitle('Reconstrução de Sinal por FFT (Edge Computing) - ESP32', fontsize=14, fontweight='bold')

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
axs[1, 0].set_title('Sinal Reconstruído no Tempo (Sintetizado)')
axs[1, 0].set_xlabel('Tempo (ms)')
axs[1, 0].set_ylabel('Amplitude Sintetizada (Raw)')
axs[1, 0].set_xlim(0, 50)  # Visualizando 50 milissegundos de onda 
axs[1, 0].set_ylim(0, 2500) # Limite ajustado para o range real do ADC (0 a 4095)

# 4. Corrente
linha_corrente, = axs[1, 1].plot([], [], 'm-', linewidth=2)
axs[1, 1].set_title('Corrente (A)')
axs[1, 1].set_xlim(0, MAX_PONTOS)
axs[1, 1].set_ylim(0, 2)

def on_message(client, userdata, msg):
    global ultimas_harmonicas, ultimo_mean_raw
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        
        # Atualiza o Offset DC
        if "mean_raw" in payload:
            ultimo_mean_raw = payload.get("mean_raw", 0)
        
        # Processamento das métricas escalares
        if "rms_mv" in payload:
            rms_mv = payload.get("rms_mv", 0)
            corrente_a = rms_mv * FATOR_CALIBRACAO
            
            dados_rms_mv.append(rms_mv)
            dados_hz.append(payload.get("mains_hz", 0))
            dados_corrente.append(corrente_a)
            print(f"Corrente: {corrente_a:.3f} A | RMS: {rms_mv} mV | DC: {ultimo_mean_raw}")
        
        # Processamento das Frequências (FFT)
        if "harmonicas" in payload:
            ultimas_harmonicas = payload.get("harmonicas", [])
            
    except json.JSONDecodeError:
        print("Erro ao decodificar JSON")

# Corrigido o Warning do MQTT utilizando a nova API Version 2
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.subscribe(TOPICO)
client.loop_start()

def atualizar_grafico(frame):
    # Atualiza métricas simples
    linha_rms_mv.set_data(range(len(dados_rms_mv)), dados_rms_mv)
    linha_hz.set_data(range(len(dados_hz)), dados_hz)
    linha_corrente.set_data(range(len(dados_corrente)), dados_corrente)
    
    # Reconstrói a onda se houver dados de FFT
    if ultimas_harmonicas:
        TAMANHO_FFT = 1024 
        
        # Cria um vetor de tempo de 0 a 50 milissegundos
        t = np.linspace(0, 0.05, 500) 
        
        # Inicia a onda já no nível de tensão média (Offset DC) com valores float
        onda_reconstruida = np.full_like(t, ultimo_mean_raw, dtype=float)
        
        # Soma as harmônicas
        for h in ultimas_harmonicas:
            freq = h.get("f", 0)
            amp_bruta = h.get("a", 0)
            fase = h.get("p", 0)
            
            # --- NOVA NORMALIZAÇÃO DA AMPLITUDE ---
            # 1. Divide pelo tamanho da FFT (a fft_radix2 em C não divide por N)
            amp_real = amp_bruta / TAMANHO_FFT
            
            # 2. Multiplica por 2 porque a energia da senoide foi dividida 
            # entre frequências positivas e negativas (a função FFT gera espelhamento)
            amp_real = amp_real * 2.0
            
            # 3. Multiplica por 2.0 para compensar a perda de energia da Janela de Hanning
            amp_real = amp_real * 2.0
            
            # Equação da onda (Série de Fourier)
            onda_reconstruida += amp_real * np.cos(2 * np.pi * freq * t + fase)
            
        # O eixo X do gráfico mostrará o tempo em milissegundos (t * 1000)
        linha_sinal.set_data(t * 1000, onda_reconstruida)
        
    # RETORNO OBRIGATÓRIO (A falta desta linha causou o seu erro de Runtime)
    return linha_rms_mv, linha_hz, linha_sinal, linha_corrente

# --- Função para fechamento limpo ---
def ao_fechar(event):
    print("Encerrando a conexão MQTT e fechando os gráficos...")
    client.loop_stop()
    client.disconnect()
    try:
        ani.event_source.stop()
    except Exception:
        pass

# Conecta o evento de fechar a janela à nossa função
fig.canvas.mpl_connect('close_event', ao_fechar)

ani = FuncAnimation(fig, atualizar_grafico, blit=True, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()