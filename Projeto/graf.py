import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import json

# Configuração da figura
fig, ax = plt.subplots()
plt.ion() # Modo interativo

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    
    # 1. Imprimir no terminal
    print(f"Corrente RMS: {data['rms']:.2f} A")
    
    # 2. Atualizar o gráfico de espectro
    freqs = [h['f'] for h in data['freqs']]
    mags = [h['m'] for h in data['freqs']]
    
    ax.clear()
    ax.bar(freqs, mags, color='cyan')
    ax.set_title("Espectro de Frequências (Harmônicas)")
    ax.set_xlabel("Frequência (Hz)")
    ax.set_ylabel("Magnitude")
    plt.draw()
    plt.pause(0.01)

client = mqtt.Client()
client.on_message = on_message
client.connect("10.0.105.207", 1883)
client.subscribe("esp32/espectro")
client.loop_forever()