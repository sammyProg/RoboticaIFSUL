import cv2
import numpy as np
import time

# =================================================================
#                         CONFIGURAÇÕES
# =================================================================

# 1. SUBSTITUA PELO SEU IP DO MOMENTO (Olhe no app IP Webcam)
url = "yourIP"  

# 2. SUA MEDIDA REAL (em metros)
TAMANHO_QUADRADO = 0.023  

# 3. Configuração do Tabuleiro (Cantos Internos)
DIMENSOES_TABULEIRO = (9, 6)

# =================================================================

# Prepara os pontos 3D padrão do tabuleiro (0,0,0), (1,0,0), (2,0,0)...
objp = np.zeros((DIMENSOES_TABULEIRO[0] * DIMENSOES_TABULEIRO[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:DIMENSOES_TABULEIRO[0], 0:DIMENSOES_TABULEIRO[1]].T.reshape(-1, 2)
objp = objp * TAMANHO_QUADRADO

# Listas para armazenar os pontos
objpoints = [] # Pontos 3D no mundo real
imgpoints = [] # Pontos 2D na imagem

print(f"Tentando conectar em: {url}")
cap = cv2.VideoCapture(url)

# Configurações para reduzir o LAG
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 

# Configura a janela
cv2.namedWindow('Calibracao Otimizada', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Calibracao Otimizada', 800, 600)

print("\n--- INSTRUÇÕES ---")
print("1. Aponte para o tabuleiro na tela.")
print("2. Quando as linhas coloridas aparecerem, aperte 'S' para salvar.")
print("3. Tire cerca de 15 a 20 fotos em posições variadas.")
print("4. Aperte 'ESC' (ou feche a janela) para finalizar e calcular.")
print("------------------\n")

frame_count = 0
ultimo_ret = False
ultimo_corners = None
capturas_salvas = 0

while True:
    # OTIMIZAÇÃO DE BUFFER:
    # .grab() joga fora o quadro antigo para garantir que .retrieve() pegue o atual
    cap.grab()
    ret, frame = cap.retrieve()
    
    if not ret:
        print("Erro ao receber imagem. Verifique o IP ou Wi-Fi.")
        break

    frame_count += 1
    display_frame = frame.copy()
    
    # Converte para escala de cinza
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # --- LÓGICA DE PROCESSAMENTO ---
    # Só processa o xadrez a cada 5 quadros para não travar o vídeo (CPU Friendly)
    if frame_count % 5 == 0:
        ret_chess, corners = cv2.findChessboardCorners(gray, DIMENSOES_TABULEIRO, None)
        
        if ret_chess:
            # Melhora a precisão dos cantos (Subpixel)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            
            # Atualiza as variáveis globais para desenhar
            ultimo_ret = True
            ultimo_corners = corners2
        else:
            ultimo_ret = False
    
    # --- DESENHO NA TELA ---
    # Se encontrou (neste quadro ou no recente), desenha
    if ultimo_ret and ultimo_corners is not None:
        cv2.drawChessboardCorners(display_frame, DIMENSOES_TABULEIRO, ultimo_corners, True)
        cv2.putText(display_frame, "ENCONTRADO! Aperte 'S'", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.putText(display_frame, f"Salvas: {capturas_salvas}", (20, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow('Calibracao Otimizada', display_frame)

    # --- CONTROLES ---
    k = cv2.waitKey(1)

    # Fecha se apertar ESC ou clicar no X da janela
    if k == 27 or cv2.getWindowProperty('Calibracao Otimizada', cv2.WND_PROP_VISIBLE) < 1:
        print("Encerrando captura...")
        break
    
    # Salva captura com 'S'
    if k == ord('s') and ultimo_ret:
        objpoints.append(objp)
        imgpoints.append(ultimo_corners)
        capturas_salvas += 1
        print(f"--> Captura {capturas_salvas} salva!")
        time.sleep(0.2) # Pausa rápida para não salvar duplicado

cap.release()
cv2.destroyAllWindows()

# --- CÁLCULO FINAL DA CALIBRAÇÃO ---
if capturas_salvas > 10:
    print("\nCalculando parâmetros da câmera... Aguarde...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n=== SUCESSO! ===")
    print(f"Erro médio (reprojeção): {ret:.4f} (Ideal é abaixo de 1.0)")
    
    # Salva o arquivo importante
    np.savez("calibracao_camera.npz", mtx=mtx, dist=dist)
    print("Arquivo 'calibracao_camera.npz' salvo na pasta do projeto.")
    print("Agora você pode medir distâncias!")
else:
    print("\nATENÇÃO: Poucas fotos foram tiradas.")

    print("O arquivo não foi gerado. Rode novamente e tire pelo menos 10 fotos.")
