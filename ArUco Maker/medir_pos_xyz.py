import cv2
import cv2.aruco as aruco
import numpy as np
import time
import csv
import os
from datetime import datetime
from collections import deque

# =================================================================
# CONFIGURAÇÕES DE ALINHAMENTO (MATLAB DH)
# =================================================================
FONTE_VIDEO = "YourIP"
ARQUIVO_CALIBRACAO = "calibracao_camera.npz"
NOME_ARQUIVO_LOG = "dados_final_limpos.csv"

ID_BASE = 0
ID_EFETUADOR = 1
TAMANHO_MARCADOR = 0.032 

# Calibração baseada nos seus dados para atingir (0, 54, 87.3)
OFFSET_X, OFFSET_Y, OFFSET_Z = -11.5, 11.5, -5.2 
MULT_Y = -100.0

# Limite de salto (em cm): Se mudar mais que 20cm entre frames, é erro de leitura.
LIMITE_SALTO = 20.0 
INTERVALO_GRAVACAO = 1.0 # Gravar a cada 1 segundo

# =================================================================
# INICIALIZAÇÃO
# =================================================================
if not os.path.exists(ARQUIVO_CALIBRACAO):
    print(f"Erro: Arquivo {ARQUIVO_CALIBRACAO} não encontrado.")
    exit()

dados = np.load(ARQUIVO_CALIBRACAO)
mtx, dist = dados['mtx'], dados['dist']
detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_5X5_1000), aruco.DetectorParameters())

buffer_x = deque(maxlen=10)
buffer_y = deque(maxlen=10)
buffer_z = deque(maxlen=10)

ultima_posicao_valida = None
proximo_tempo_gravacao = time.time()
contador_pontos = 1

def get_pose(corner):
    obj_p = np.array([[-TAMANHO_MARCADOR/2, TAMANHO_MARCADOR/2, 0],
                      [TAMANHO_MARCADOR/2, TAMANHO_MARCADOR/2, 0],
                      [TAMANHO_MARCADOR/2, -TAMANHO_MARCADOR/2, 0],
                      [-TAMANHO_MARCADOR/2, -TAMANHO_MARCADOR/2, 0]], dtype=np.float32)
    _, rvec, tvec = cv2.solvePnP(obj_p, corner, mtx, dist)
    return rvec, tvec

# =================================================================
# LOOP PRINCIPAL COM FILTRO E GRAVAÇÃO CSV
# =================================================================
cap = cv2.VideoCapture(FONTE_VIDEO)

# Abrir arquivo para escrita (modo 'a' permite continuar de onde parou)
with open(NOME_ARQUIVO_LOG, mode='a', newline='') as f:
    writer = csv.writer(f)
    
    # Escrever cabeçalho apenas se o arquivo for novo
    if os.stat(NOME_ARQUIVO_LOG).st_size == 0:
        writer.writerow(['Data/Hora', 'ID', 'X_DH', 'Y_DH', 'Z_DH'])

    print(f"Iniciado. Gravando em: {NOME_ARQUIVO_LOG}")

    while True:
        ret, frame = cap.read()
        if not ret: break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and ID_BASE in ids and ID_EFETUADOR in ids:
            ids_l = ids.flatten().tolist()
            r_b, t_b = get_pose(corners[ids_l.index(ID_BASE)])
            r_e, t_e = get_pose(corners[ids_l.index(ID_EFETUADOR)])

            R_b, _ = cv2.Rodrigues(r_b)
            p_rel = R_b.T @ (t_e - t_b)
            
            # Coordenadas Brutas
            cx = p_rel[0][0] * 100 + OFFSET_X
            cy = p_rel[1][0] * MULT_Y + OFFSET_Y
            cz = p_rel[2][0] * 100 + OFFSET_Z

            # --- FILTRO DE REJEIÇÃO DE OUTLIERS ---
            pos_atual = np.array([cx, cy, cz])
            
            validar_frame = True
            if ultima_posicao_valida is not None:
                distancia_salto = np.linalg.norm(pos_atual - ultima_posicao_valida)
                if distancia_salto > LIMITE_SALTO:
                    validar_frame = False # Pula este frame ruidoso

            if validar_frame:
                ultima_posicao_valida = pos_atual
                buffer_x.append(cx)
                buffer_y.append(cy)
                buffer_z.append(cz)

                # Média suavizada
                xf, yf, zf = np.mean(buffer_x), np.mean(buffer_y), np.mean(buffer_z)

                # --- LÓGICA DE GRAVAÇÃO TEMPORIZADA ---
                if time.time() >= proximo_tempo_gravacao:
                    agora = datetime.now().strftime("%H:%M:%S")
                    writer.writerow([agora, f"P_{contador_pontos}", round(xf, 2), round(yf, 2), round(zf, 2)])
                    f.flush() # Força a escrita no arquivo
                    
                    print(f"Ponto {contador_pontos} salvo: X={xf:.1f} Y={yf:.1f} Z={zf:.1f}")
                    
                    contador_pontos += 1
                    proximo_tempo_gravacao = time.time() + INTERVALO_GRAVACAO

                # Exibição na tela
                cv2.putText(frame, f"DH: {xf:.1f}, {yf:.1f}, {zf:.1f}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Filtro de Pose Robusto", frame)
        if cv2.waitKey(1) == ord('q'): break

cap.release()
cv2.destroyAllWindows()