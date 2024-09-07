clc
clear all
close all
% Carregar os dados do arquivo CSV e preservar os nomes das colunas originais
data = readtable('results.csv', 'VariableNamingRule', 'preserve');

% Gerar o vetor de tempo (frequência de amostragem de 10 Hz)
Fs = 10;  % Frequência de amostragem de 10 Hz
n_samples = height(data);  % Número de amostras
time = (0:n_samples-1) / Fs;  % Vetor de tempo, em segundos

% Configurar os dados
angular_velocity_x = data{:, '/imu/angular_velocity/x'};
angular_velocity_y = data{:, '/imu/angular_velocity/y'};
angular_velocity_z = data{:, '/imu/angular_velocity/z'};
linear_acceleration_x = data{:, '/imu/linear_acceleration/x'};
linear_acceleration_y = data{:, '/imu/linear_acceleration/y'};
linear_acceleration_z = data{:, '/imu/linear_acceleration/z'};

% Configurar estilo para artigo científico
set(0, 'DefaultAxesFontSize', 14);
set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultTextInterpreter', 'latex');  % Configurar latex como interpretador para todo o texto
set(0, 'DefaultLegendInterpreter', 'latex');  % Configurar latex para legendas
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');  % Configurar latex para rótulos dos ticks

% Definir a paleta de cores para os gráficos
colors = lines(3);  % Usar uma paleta de cores mais diferenciada (3 cores)

%% Plot para aceleração linear (Aceleração X, Y, Z)
figure;
hold on;
grid on;
plot(time, linear_acceleration_x, 'Color', colors(1, :), 'DisplayName', '$a_x$');
plot(time, linear_acceleration_y, 'Color', colors(2, :), 'DisplayName', '$a_y$');
plot(time, linear_acceleration_z, 'Color', colors(3, :), 'DisplayName', '$a_z$');
title('Aceleração Linear', 'Interpreter', 'latex');  % Adicionando título com LaTeX
xlabel('Tempo (s)', 'Interpreter', 'latex');
ylabel('Aceleração (m/s$^2$)', 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'best');
axis square;
grid minor;

% Salvar o gráfico
saveas(gcf, 'aceleracao_linear.png');

%% Plot para velocidade angular (Velocidade Angular X, Y, Z)
figure;
hold on;
grid on;
plot(time, angular_velocity_x, 'Color', colors(1, :), 'DisplayName', '$\omega_x$');
plot(time, angular_velocity_y, 'Color', colors(2, :), 'DisplayName', '$\omega_y$');
plot(time, angular_velocity_z, 'Color', colors(3, :), 'DisplayName', '$\omega_z$');
title('Velocidade Angular', 'Interpreter', 'latex');  % Adicionando título com LaTeX
xlabel('Tempo (s)', 'Interpreter', 'latex');
ylabel('Velocidade (rad/s)', 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'best');
axis square;
grid minor;

% Salvar o gráfico
saveas(gcf, 'velocidade_angular.png');
