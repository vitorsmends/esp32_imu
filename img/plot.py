import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Configuração do tema do seaborn
sns.set_theme(style="whitegrid")

# Definir tamanho da fonte para os gráficos
font_size = 14  # Ajuste o tamanho da fonte aqui

# Carregar dados do arquivo CSV
data = pd.read_csv('results.csv')

# Definir a frequência de amostragem e calcular o período
sampling_frequency = 14  # em Hz
sampling_period = 1 / sampling_frequency  # em segundos

# Gerar o eixo de tempo
num_samples = len(data)
time = np.arange(0, num_samples * sampling_period, sampling_period)

# Extrair variáveis de interesse
omega_x = data['/imu/angular_velocity/x']
omega_y = data['/imu/angular_velocity/y']
omega_z = data['/imu/angular_velocity/z']
acc_x = data['/imu/linear_acceleration/x']
acc_y = data['/imu/linear_acceleration/y']
acc_z = data['/imu/linear_acceleration/z']

# Criar a primeira figura para a velocidade angular
plt.figure(figsize=(8, 4))

# Gráfico de Velocidade Angular
plt.plot(time, omega_x, 'r', label=r'$\omega_x$')
plt.plot(time, omega_y, 'g', label=r'$\omega_y$')
plt.plot(time, omega_z, 'b', label=r'$\omega_z$')
plt.xlabel('Tempo (s)', fontsize=font_size)
plt.ylabel(r'Velocidade Angular (rad/s)', fontsize=font_size)
plt.legend(fontsize=font_size)
plt.grid(True)

# Ajustar o layout da figura
plt.tight_layout()
plt.savefig('angular_velocity.png')
plt.show()

# Criar a segunda figura para a aceleração linear
plt.figure(figsize=(8, 4))

# Gráfico de Aceleração Linear
plt.plot(time, acc_x, 'r', label=r'$a_x$')
plt.plot(time, acc_y, 'g', label=r'$a_y$')
plt.plot(time, acc_z, 'b', label=r'$a_z$')
plt.xlabel('Tempo (s)', fontsize=font_size)
plt.ylabel(r'Aceleração Linear (m/s$^2$)', fontsize=font_size)
plt.legend(fontsize=font_size)
plt.grid(True)

# Ajustar o layout da figura
plt.tight_layout()
plt.savefig('linear_acceleration.png')
plt.show()
