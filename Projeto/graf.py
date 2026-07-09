import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. DADOS DO LOG DA ESP32 (OFFLINE)
# ==========================================
MEAN_RAW = 886.47
RMS_RAW = 1029.42
RMS_MV = 829.2
FS = 4000
N_SAMPLES = 1024

# Fator de calibração para converter o valor ADC Raw para Amperes 
# (Ajuste este valor de acordo com os testes do seu sensor ZMCT103C)
FATOR_CALIBRACAO_CORRENTE = 0.005 

# Lista de todas as 20 harmónicas (Frequência, Magnitude, Fase em Radianos) extraídas do seu log
harmonicas_log = [
    (62.5, 321724.66, -0.29),
    (125.0, 86702.82, -0.58),
    (238.3, 44518.94, 1.69),
    (183.6, 37857.94, -0.89),
    (359.4, 26078.45, 1.53),
    (421.9, 16917.84, 0.53),
    (296.9, 12751.35, -2.83),
    (484.4, 10706.68, -1.50),
    (546.9, 8222.18, -2.76),
    (316.4, 6456.35, -0.33),
    (449.2, 4723.79, 2.95),
    (609.4, 4621.43, 2.02),
    (660.2, 4358.92, -0.11),
    (714.8, 4104.82, -0.29),
    (777.3, 3638.59, -1.14),
    (898.4, 3271.91, -1.09),
    (1496.1, 2238.46, 1.98),
    (1621.1, 1978.50, -1.49),
    (1382.8, 1913.08, -1.08),
    (1683.6, 1877.45, -2.28)
]

# ==========================================
# 2. GERAÇÃO DO EIXO TEMPORAL E RECONSTRUÇÃO
# ==========================================
t = np.arange(N_SAMPLES) / FS

# Inicia a onda com o valor de Offset DC (Centro da onda)
sinal_reconstruido = np.ones(N_SAMPLES) * MEAN_RAW

# Soma cada uma das 20 senoides no domínio do tempo
for freq, mag, fase in harmonicas_log:
    # Correção da magnitude da FFT para a Amplitude Real no tempo
    amplitude = (2.0 * mag) / N_SAMPLES
    # Síntese pela equação de Euler (Série Cossenoidal)
    sinal_reconstruido += amplitude * np.cos(2 * np.pi * freq * t + fase)

# ==========================================
# 3. IMPRESSÃO NO TERMINAL
# ==========================================
corrente_estimada = RMS_RAW * FATOR_CALIBRACAO_CORRENTE

print("-" * 50)
print(f"📡 DADOS CALCULADOS (OFFLINE - 20 HARMÓNICAS)")
print("-" * 50)
print(f" Nível DC (Offset) : {MEAN_RAW} raw")
print(f" Tensão RMS (ESP)  : {RMS_MV} mV")
print(f" Valor RMS Raw     : {RMS_RAW} raw")
print(f" Corrente Estimada : {corrente_estimada:.2f} A")
print("-" * 50)

# ==========================================
# 4. PLOTAGEM DO GRÁFICO
# ==========================================
plt.figure(figsize=(12, 6))

# Define quantos ms queremos visualizar (ex: 60ms para mostrar cerca de 3 a 4 ciclos da rede 60Hz)
amostras_visiveis = int(FS * 0.060) 

plt.plot(t[:amostras_visiveis] * 1000, sinal_reconstruido[:amostras_visiveis], 
         color='firebrick', linewidth=2.5, label='Sinal Reconstruído (20 Harmónicas)')

plt.axhline(MEAN_RAW, color='gray', linestyle='--', label=f'Nível DC ({MEAN_RAW})')

# Detalhes visuais
plt.title("Reconstrução de Alta Fidelidade da Corrente (ESP-DSP)", fontsize=14)
plt.xlabel("Tempo (ms)", fontsize=12)
plt.ylabel("Amplitude (ADC Raw)", fontsize=12)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='upper right')
plt.tight_layout()

# Exibe a janela gráfica
plt.show()