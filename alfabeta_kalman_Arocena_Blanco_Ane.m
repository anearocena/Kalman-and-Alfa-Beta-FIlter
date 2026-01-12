%% ---------------------------------------------------
%               Parámetros Generales
%% ---------------------------------------------------

clear; clc; close all; 
% (Borro todo: variables, consola y figuras, así no hay nada de ejecuciones anteriores que me moleste)

% Paso de nudos a metros/segundo para poder operar luego todo bien
conv_knots_to_mps = 1852/3600;   % 1 nudo = 1852 m / 3600 s   
velocidad_knots   = 250;         % La velocidad dada es 250 nudos

% Consigo ahora una variable v_mps para cuando haga v_mps* dt me de los metros recorridos en cada dt

v_mps = velocidad_knots * conv_knots_to_mps;  

% — Defino el intervalo de muestreo del radar SSR —
dt = 4;  % Cada 4 segundos obtenemos una medida SSR nueva

% — Ruido de medición SSR —
sigma_rho   = 200;       % Desviación típica del error en la distancia medida (en metros)
sigma_alpha = deg2rad(0.3);  % Desviación típica del error angular (0.3 grados pasados a radianes, porque matlab no trabaja con grados)


%..................................................
% — Parámetros para el filtro α–β —
%..................................................

sigma_s = 250;                          % Ruido de proceso supuesto, en metros
rho_num  = sigma_s^2; 
rho_den  = (sigma_rho^2) * dt;
rho      = rho_num / rho_den;           % Se usa para calcular alpha y beta

aux_a   = 8 * rho^2 + 1;
a_term  = sqrt(aux_a);                  % Auxiliar para las fórmulas

num_alpha = -4*rho^2 + 8*rho + 1 - a_term;
den_alpha = 4*rho + 1;
alpha     = num_alpha / den_alpha;  % Ganancia que uso para corregir la posición


num_beta = 4 * (2*rho^2 + 1 + a_term);
den_beta = (4*rho + 1)^2;
beta     = num_beta / den_beta;      % Ganancia para corregir la velocidad

%..................................................
% — Parámetros para el filtro de Kalman —
%..................................................

% Estado = [x; y; vx; vy]
P0 = [50000 0     0    0;
       0   50000  0    0;
       0    0   500   0;
       0    0    0   500];

% Matriz de covarianza inicial dada en el enunciado con valores elevados ya
% que al principio no tengo una información exacta de la posición, pero va
% mejorando a medida que converge

sigma_sp = 250;  % desviación de proceso (posición) en m
sigma_sv = 0.1;  % desviación de proceso (velocidad) en m/s

Q = [ sigma_sp^2   0           0         0;
       0         sigma_sp^2    0         0;
       0           0       sigma_sv^2    0;
       0           0           0     sigma_sv^2 ];
% Matriz Q de covarianza del ruido de proceso para Kalman

R = [ sigma_rho^2   0;
       0         sigma_rho^2 ];
% Matriz R de covarianza del ruido de medida en X e Y 
% (supongo que error en X = sigma_rho y en Y = sigma_rho)
% Nota: técnicamente el SSR mide polar, pero aproximamos a cartesianas

%% ---------------------------------------------------
%            1) Generación de la Trayectoria
%% ---------------------------------------------------

% La idea es construir la pista “de hipódromo” con 5 fases: 
%  1) tramo recto de aproximación
%  2) un semicírculo
%  3) tramo recto de alejamiento
%  4) segundo semicírculo
%  5) tramo recto con la aproximación final

% — 1) Tramo de aproximación radial (90°) desde 35000m a 13000m —

% Punto inicial:
x0 = 35000; 
y0 = 0; 
theta0 = pi;  
% Dirección = pi radianes = 180° (se mueve hacia el oeste)

% Distancia total de ese tramo:
dist1 = 35000 - 13000;  
% = 22000 m

% Número de puntos en ese tramo, cuantas muestras vamos a tener:
dist_por_paso = v_mps * dt;   
% Cuánto avanza en metros cada vez que SSR actualiza (v*dt)
N1 = ceil(dist1 / dist_por_paso);
% Redondeo hacia arriba para asegurarme de tener suficientes muestras

% Construyo un vector de distancias desde 0 a dist1 (1×N1)
s_vector1 = linspace(0, dist1, N1);

% Ahora, para cada valor de s en s_vector1, calculo 
% x = x0 + s*cos(theta0),   y = y0 + s*sin(theta0)
seg1_x = zeros(1, N1);
seg1_y = zeros(1, N1);
for i = 1:N1
    s_i = s_vector1(i);
    seg1_x(i) = x0 + s_i * cos(theta0);
    seg1_y(i) = y0 + s_i * sin(theta0);
end
% Así obtengo N1 puntos que van desde (35000,0) hasta (13000,0)

% El “punto de giro 1” es el último punto de seg1:
p1_x = seg1_x(end);
p1_y = seg1_y(end);

% — 2) Viraje de 180° a derechas con radio 5000m —

R1 = 5000;  
% Radio del semicírculo
% Centro del semicírculo = 5000 m “arriba” del punto de giro
c1_x = p1_x + 0;  
c1_y = p1_y + R1;

% Para dar media vuelta, recorro un arco de longitud pi*R1.
long_arco1 = pi * R1;
N2 = ceil(long_arco1 / dist_por_paso);

% Genero un vector de ángulos: 
% Quiero empezar en -pi/2 (debajo del centro) y terminar en +pi/2 (arriba del centro)
ang_init1 = -pi/2;
ang_fin1  = -pi/2 - pi;  
angulos2 = linspace(ang_init1, ang_fin1, N2);
% Así me aseguro de recorrer los π radianes necesarios

seg2_x = zeros(1, N2);
seg2_y = zeros(1, N2);
for i = 1:N2
    a_i = angulos2(i);
    seg2_x(i) = c1_x + R1 * cos(a_i);
    seg2_y(i) = c1_y + R1 * sin(a_i);
end
% Esto dibuja los N2 puntos del semicírculo en sentido horario 
% porque arranca en -pi/2 y sube hasta pi/2

% Punto de giro 2: final de seg2
p2_x = seg2_x(end);
p2_y = seg2_y(end);

% — 3) Tramo de alejamiento recto de 7000m —

x3_0 = p2_x;
y3_0 = p2_y;
theta3 = 0;  
% Dirección 0 radianes = 0° (hacia el este)

dist3 = 7000;  
N3    = ceil(dist3 / dist_por_paso);
s_vector3 = linspace(0, dist3, N3);

seg3_x = zeros(1, N3);
seg3_y = zeros(1, N3);
for i = 1:N3
    s_i = s_vector3(i);
    seg3_x(i) = x3_0 + s_i * cos(theta3);
    seg3_y(i) = y3_0 + s_i * sin(theta3);
end
% Con esto tengo la parte recta de 7000 m hacia el este

% Punto de giro 3:
p3_x = seg3_x(end);
p3_y = seg3_y(end);

% — 4) Segundo viraje de 180° a derechas, radio 5000m —

R2 = 5000;
c2_x = p3_x + 0;
c2_y = p3_y - R2;  
% Ahora el centro está 5000 m “debajo” de (p3_x, p3_y)

long_arco2 = pi * R2;
N4 = ceil(long_arco2 / dist_por_paso);

% El punto inicial del semicírculo está “arriba” (ángulo +pi/2),
% y gira hasta ángulo -pi/2
ang_init2 = +pi/2;
ang_fin2  = -pi/2;  
angulos4 = linspace(ang_init2, ang_fin2, N4);

seg4_x = zeros(1, N4);
seg4_y = zeros(1, N4);
for i = 1:N4
    a_i = angulos4(i);
    seg4_x(i) = c2_x + R2 * cos(a_i);
    seg4_y(i) = c2_y + R2 * sin(a_i);
end
% Esto crea el segundo semicírculo en sentido horario

% Punto de giro 4:
p4_x = seg4_x(end);
p4_y = seg4_y(end);

% — 5) Tramo de aproximación radial de 14000m —

x5_0 = p4_x;
y5_0 = p4_y;
theta5 = pi;  % Dirección 180° (hacia el oeste)

dist5 = 14000;
N5    = ceil(dist5 / dist_por_paso);
s_vector5 = linspace(0, dist5, N5);

seg5_x = zeros(1, N5);
seg5_y = zeros(1, N5);
for i = 1:N5
    s_i = s_vector5(i);
    seg5_x(i) = x5_0 + s_i * cos(theta5);
    seg5_y(i) = y5_0 + s_i * sin(theta5);
end
% Último tramo de 14000 m de regreso hacia el oeste

% — Concatenar todos los segmentos en un único vector de posiciones —

X_all = [ seg1_x, seg2_x, seg3_x, seg4_x, seg5_x ];
Y_all = [ seg1_y, seg2_y, seg3_y, seg4_y, seg5_y ];

num_total = length(X_all);  
% Total de puntos = N1+N2+N3+N4+N5

% Ahora creo la matriz X_true (num_total × 2) 
% donde cada fila es [x, y]
X_true = zeros(num_total, 2);
for i = 1:num_total
    X_true(i,1) = X_all(i);
    X_true(i,2) = Y_all(i);
end

% Vector de tiempos t = [0; dt; 2dt; …]
t = zeros(num_total, 1);
for i = 1:num_total
    t(i) = (i-1) * dt;
end


%% ---------------------------------------------------
%      2) Simulación de Mediciones SSR (polar → cartesiano)
%% ---------------------------------------------------

% Calculo primero el rango verdadero punto a punto
r_true = zeros(num_total, 1);
for i = 1:num_total
    xx = X_true(i,1);
    yy = X_true(i,2);
    r_true(i) = sqrt(xx^2 + yy^2);
end
% r_true tiene la distancia real al radar en cada instante

% Calculo el ángulo verdadero (theta) usando atan2 (devuelve radianes)
ang_true = zeros(num_total, 1);
for i = 1:num_total
    xx = X_true(i,1);
    yy = X_true(i,2);
    ang_true(i) = atan2(yy, xx);
end
% ang_true tiene el ángulo real para cada punto

% Simulación de ruido en polar (r + θ)
r_meas   = zeros(num_total, 1);
ang_meas = zeros(num_total, 1);
for i = 1:num_total
    ruido_rho  = sigma_rho   * randn;  
    ruido_ang  = sigma_alpha * randn;
    r_meas(i)   = r_true(i)   + ruido_rho;
    ang_meas(i) = ang_true(i) + ruido_ang;
end
% Ahora r_meas y ang_meas son los valores medidos con ruido

% Paso a coordenadas cartesianas: x = r*cos(theta), y = r*sin(theta)
Z = zeros(num_total, 2);
for i = 1:num_total
    Z(i,1) = r_meas(i) * cos(ang_meas(i));
    Z(i,2) = r_meas(i) * sin(ang_meas(i));
end
% Z es la matriz de mediciones SSR en cartesianas, fila a fila


%% ---------------------------------------------------
%     3) Filtro α–β (dos bucles: eje X y eje Y)
%% ---------------------------------------------------

% Preasigno matrices para guardarme la estimación
x_ab = zeros(num_total, 2);  
v_ab = zeros(num_total, 2);

% Inicializo la primera fila (t = 0) con la primera medición y velocidad 0
x_ab(1,1) = Z(1,1);  % posición estimada X(0) = primera medida X
x_ab(1,2) = Z(1,2);  % posición estimada Y(0) = primera medida Y
v_ab(1,1) = 0;       % velocidad estimada X(0) = 0
v_ab(1,2) = 0;       % velocidad estimada Y(0) = 0

% Itero desde k=2 hasta k = num_total
for k = 2:num_total
    
    % --- Eje X (ax = 1) ---
    % Predicción de posición en X: x_pred = x_prev + vx_prev * dt
    x_prev     = x_ab(k-1, 1);
    vx_prev    = v_ab(k-1, 1);
    x_pred_x   = x_prev + vx_prev * dt;
    v_pred_x   = vx_prev;
    
    % Medida X en t=k
    z_x        = Z(k,1);
    
    % Innovación (error de predicción en X):
    innov_x    = z_x - x_pred_x;
    
    % Corrección en X (aplico α–β):
    x_ab(k,1)  = x_pred_x + alpha * innov_x;
    v_ab(k,1)  = v_pred_x + (beta / dt) * innov_x;
    
    
    % --- Eje Y (ax = 2) ---
    % Predicción de posición en Y: y_pred = y_prev + vy_prev * dt
    y_prev     = x_ab(k-1, 2);
    vy_prev    = v_ab(k-1, 2);
    y_pred_y   = y_prev + vy_prev * dt;
    v_pred_y   = vy_prev;
    
    % Medida Y en t=k
    z_y        = Z(k,2);
    
    % Innovación (error de predicción en Y):
    innov_y    = z_y - y_pred_y;
    
    % Corrección en Y (aplico α–β):
    x_ab(k,2)  = y_pred_y + alpha * innov_y;
    v_ab(k,2)  = v_pred_y + (beta / dt) * innov_y;
    
end
% Al terminar, x_ab(:,1) y x_ab(:,2) tienen las posiciones filtradas 
% para cada instante, y v_ab(:,1), v_ab(:,2) tienen velocidades filtradas


%% ---------------------------------------------------
%    4) Filtro de Kalman (estado = [x; y; vx; vy])
%% ---------------------------------------------------

% Preasigno matrices
xk = zeros(4, num_total);
Pk = zeros(4, 4, num_total);

% Condición inicial de estado y covarianza
xk(1,1) = Z(1,1);   % x(0) = primer medido X
xk(2,1) = Z(1,2);   % y(0) = primer medido Y
xk(3,1) = 0;        % vx(0) = 0
xk(4,1) = 0;        % vy(0) = 0

Pk(:,:,1) = P0;     % P(0) = P0, que definimos arriba

% Matriz de transición de estado F (4×4)
F = [ 1  0   dt  0;
      0  1    0  dt;
      0  0    1   0;
      0  0    0   1 ];

% Matriz de observación H (2×4), sólo mapeamos [x;y]
H = [ 1  0  0  0;
      0  1  0  0 ];

for k = 2:num_total
    
    % 1) Predicción del estado: xk_pred = F * xk(:,k-1)
    xk_pred = F * xk(:,k-1);
    
    % 2) Predicción de la covarianza: Pk_pred = F*Pk(:,:,k-1)*F' + Q
    Pk_pred = F * Pk(:,:,k-1) * F' + Q;
    
    % 3) Tomo la medida en t=k (vector 2×1)
    z_k = [ Z(k,1); Z(k,2) ];
    
    % 4) Calculo la covarianza de la innovación: S_k = H*Pk_pred*H' + R
    S_k = H * Pk_pred * H' + R;
    
    % 5) Calculo la ganancia de Kalman: Kk = Pk_pred * H' * inv(S_k)
    Kk = Pk_pred * H' * inv(S_k);
    % Uso inv() en lugar de “/ S_k” para que se entienda mejor
    
    % 6) Actualizo el estado con la innovación
    y_tilde = z_k - H * xk_pred;      
    xk(:,k) = xk_pred + Kk * y_tilde; 
    
    % 7) Actualizo la covarianza: Pk = (I - Kk*H) * Pk_pred
    I4 = eye(4);
    Pk(:,:,k) = (I4 - Kk * H) * Pk_pred;
    
end
% Al final, xk(1,:) y xk(2,:) contienen las posiciones estimadas 
% por Kalman, y xk(3,:), xk(4,:) las velocidades estimadas


%% ---------------------------------------------------
%                  5) Dibujar resultados
%% ---------------------------------------------------

% --- 5.1) Figura para Filtro α–β ---
figure; 
hold on; 
grid on; 
axis equal;

% 1) Trayectoria teórica (línea negra)
plot(X_true(:,1), X_true(:,2), 'k-', 'LineWidth', 1.5);

% 2) Medidas SSR (puntos azules)
plot(Z(:,1), Z(:,2), '.', 'Color', [0 0 1], 'MarkerSize', 8);

% 3) Estimación α–β (línea magenta discontinua)
plot(x_ab(:,1), x_ab(:,2), 'm--', 'LineWidth', 1.2);

xlabel('X (m)');
ylabel('Y (m)');
title('Filtro α–β: trayectoria teórica vs SSR vs filtrada');
legend('Teórica','SSR','α–β','Location','best');
% Comentario: Aquí se ve la comparación entre la línea negra (teórica),
% los puntitos azules (SSR con ruido) y la línea magenta punteada (α–β).

% --- 5.2) Figura para Filtro de Kalman ---
figure; 
hold on; 
grid on; 
axis equal;

% 1) Trayectoria teórica (línea negra)
plot(X_true(:,1), X_true(:,2), 'k-', 'LineWidth', 1.5);

% 2) Medidas SSR (puntos rojos)
plot(Z(:,1), Z(:,2), '.', 'Color', [1 0 0], 'MarkerSize', 8);

% 3) Estimación Kalman (línea cian continua)
plot(xk(1,:), xk(2,:), 'c-', 'LineWidth', 1.2);

xlabel('X (m)');
ylabel('Y (m)');
title('Filtro de Kalman: trayectoria teórica vs SSR vs filtrada');
legend('Teórica','SSR','Kalman','Location','best');
% Comentario: Aquí la línea cian del filtro de Kalman se superpone muy bien a la trayectoria negra.
