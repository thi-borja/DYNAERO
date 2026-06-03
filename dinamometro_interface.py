# ============================================================
#  DINAMÔMETRO DE MOTOR E HÉLICE - RC AIRCRAFT
#  Interface Gráfica - Python
#
#  Instale as dependências:
#    pip install pyserial matplotlib
#
#  Execute:
#    python dinamometro_interface.py
# ============================================================

import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
from collections import deque
from datetime import datetime

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============================================================
#  CONFIGURAÇÕES
# ============================================================
BAUD_RATE     = 115200
JANELA_PONTOS = 300
INTERVALO_MS  = 60   # ~16 fps
RAW_TIMEOUT_S = 5.0
EMPUXO_UNIDADE = "N"
TORQUE_UNIDADE = "N.m"
SINAL_EMPUXO = -1   # Use 1 ou -1 para ajustar o sentido exibido/gravado.
SINAL_TORQUE = 1    # Use 1 ou -1 para ajustar o sentido exibido/gravado.
EMPUXO_Y_MIN = 0.0
EMPUXO_Y_MAX = 180.0
TORQUE_Y_MIN = 0.0
TORQUE_Y_MAX = 3.0
TORQUE_COR = "#4268ff"
DENSIDADE_AR_PADRAO = 1.225
LOGO_PATH = r"C:\Users\thigr\Downloads\logotipo-dynaero-noback.png"
LOGO_MAX_WIDTH = 110
LOGO_MAX_HEIGHT = 64

# ============================================================
#  ESTADO GLOBAL
# ============================================================
tempo_buf  = deque(maxlen=JANELA_PONTOS)
empuxo_buf = deque(maxlen=JANELA_PONTOS)
torque_buf = deque(maxlen=JANELA_PONTOS)
tempo_grafico_offset = 0.0
ultimo_tempo_serial = None

# Histórico completo para exportação: (tempo_s, empuxo_N, torque_N_m)
historico = []
gravando = False
gravacao_inicio_local = None
gravacao_duracao_acumulada = 0.0
densidade_ar_local = DENSIDADE_AR_PADRAO

ser         = None
lendo       = False
conectado   = False

leitura_raw_e = None
leitura_raw_t = None
raw_vazio = {"E": None, "T": None}
raw_values = {"E": None, "T": None}
raw_events = {"E": threading.Event(), "T": threading.Event()}
raw_busy = {"E": False, "T": False}
calib_pontos = {
    "E": [{"forca": None, "raw": None} for _ in range(3)],
    "T": [{"forca": None, "raw": None} for _ in range(3)],
}
raw_lock = threading.Lock()
serial_lock = threading.Lock()

def ui_call(func, *args):
    try:
        root.after(0, lambda: func(*args))
    except Exception:
        pass

def registrar_raw(tipo, valor):
    with raw_lock:
        raw_values[tipo] = valor
        raw_events[tipo].set()

def tempo_gravacao_atual():
    if gravando and gravacao_inicio_local is not None:
        return gravacao_duracao_acumulada + (time.monotonic() - gravacao_inicio_local)
    return gravacao_duracao_acumulada

def formatar_tempo(segundos):
    segundos = max(0, int(segundos))
    horas, resto = divmod(segundos, 3600)
    minutos, segundos = divmod(resto, 60)
    return f"{horas:02d}:{minutos:02d}:{segundos:02d}"

def registrar_ponto_gravacao(empuxo, torque):
    historico.append((tempo_gravacao_atual(), empuxo, torque))

def corrigir_empuxo_por_densidade(empuxo):
    return (densidade_ar_local / DENSIDADE_AR_PADRAO) * empuxo

def limpar_grafico_tempo_real():
    tempo_buf.clear()
    empuxo_buf.clear()
    torque_buf.clear()
    try:
        linha_empuxo.set_data([], [])
        linha_torque.set_data([], [])
        txt_emp.set_text("")
        txt_tor.set_text("")
        ax_emp.set_xlim(0, 1)
        ax_tor.set_xlim(0, 1)
        ax_emp.set_ylim(EMPUXO_Y_MIN, EMPUXO_Y_MAX)
        ax_tor.set_ylim(TORQUE_Y_MIN, TORQUE_Y_MAX)
        canvas.draw_idle()
    except Exception:
        pass

def zerar_tempo_grafico():
    global tempo_grafico_offset
    tempo_grafico_offset = ultimo_tempo_serial if ultimo_tempo_serial is not None else 0.0
    limpar_grafico_tempo_real()
    log_status("Tempo do gráfico em tempo real zerado.")

# ============================================================
#  THREAD DE LEITURA SERIAL
# ============================================================
def ler_serial():
    global lendo, conectado, leitura_raw_e, leitura_raw_t, ultimo_tempo_serial
    while lendo:
        try:
            linha = ser.readline().decode("utf-8", errors="ignore").strip()
            if not linha:
                continue

            if linha.startswith("DATA:"):
                partes = linha.replace("DATA:", "").split(",")
                if len(partes) == 3:
                    t_serial = float(partes[0])
                    ultimo_tempo_serial = t_serial
                    t = max(0.0, t_serial - tempo_grafico_offset)
                    e_medido = float(partes[1]) * SINAL_EMPUXO
                    e = corrigir_empuxo_por_densidade(e_medido)
                    tr = float(partes[2]) * SINAL_TORQUE
                    tempo_buf.append(t)
                    empuxo_buf.append(e)
                    torque_buf.append(tr)
                    if gravando:
                        registrar_ponto_gravacao(e, tr)
                    ui_call(atualizar_leituras, e, tr)

            elif linha.startswith("STATUS:"):
                msg = linha.replace("STATUS:", "")
                ui_call(log_status, msg)

            elif linha.startswith("RAW_E:"):
                valor = float(linha.replace("RAW_E:", "").strip())
                leitura_raw_e = valor
                registrar_raw("E", valor)

            elif linha.startswith("RAW_T:"):
                valor = float(linha.replace("RAW_T:", "").strip())
                leitura_raw_t = valor
                registrar_raw("T", valor)

        except Exception:
            pass

# ============================================================
#  FUNÇÕES DE COMUNICAÇÃO SERIAL
# ============================================================
def enviar_comando(cmd):
    if not ser or not ser.is_open:
        log_status("Serial não conectada.")
        return False
    try:
        with serial_lock:
            ser.write((cmd + "\n").encode())
        return True
    except Exception as e:
        log_status(f"Erro ao enviar: {e}")
        return False

def listar_portas():
    return [p.device for p in serial.tools.list_ports.comports()]

def conectar():
    global ser, lendo, conectado, tempo_grafico_offset, ultimo_tempo_serial
    porta = combo_porta.get()
    if not porta:
        messagebox.showwarning("Aviso", "Selecione uma porta serial.")
        return
    try:
        ser = serial.Serial(porta, BAUD_RATE, timeout=1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        tempo_grafico_offset = 0.0
        ultimo_tempo_serial = None
        limpar_grafico_tempo_real()
        lendo    = True
        conectado = True
        thread = threading.Thread(target=ler_serial, daemon=True)
        thread.start()
        btn_conectar.config(state="disabled")
        btn_desconectar.config(state="normal")
        lbl_status_conn.config(text="● Conectado", foreground=GREEN)
        log_status(f"Conectado em {porta}")
    except Exception as e:
        messagebox.showerror("Erro", f"Não foi possível conectar:\n{e}")

def desconectar():
    global lendo, conectado, ser
    if gravando:
        parar_gravacao()
    lendo     = False
    conectado = False
    cancelar_raw_pendentes()
    time.sleep(0.3)
    if ser and ser.is_open:
        ser.close()
    btn_conectar.config(state="normal")
    btn_desconectar.config(state="disabled")
    lbl_status_conn.config(text="● Desconectado", foreground=RED_CLR)
    log_status("Desconectado.")

def atualizar_leituras(e, tr):
    try:
        lbl_empuxo_val.config(text=f"{e:.3f}")
        lbl_torque_val.config(text=f"{tr:.3f}")
    except Exception:
        pass

def log_status(msg):
    try:
        txt_log.config(state="normal")
        txt_log.insert("end", f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n")
        txt_log.see("end")
        txt_log.config(state="disabled")
    except Exception:
        pass

# ============================================================
#  CALIBRAÇÃO
# ============================================================
def cancelar_raw_pendentes():
    with raw_lock:
        for tipo in raw_busy:
            raw_busy[tipo] = False
            raw_values[tipo] = None
            raw_events[tipo].set()

def solicitar_raw(tipo, descricao, callback):
    if not conectado or not ser or not ser.is_open:
        messagebox.showwarning("Aviso", "Conecte à porta serial primeiro.")
        return

    with raw_lock:
        if raw_busy[tipo]:
            log_status(f"Já existe uma coleta RAW {tipo} em andamento.")
            return
        raw_busy[tipo] = True
        raw_values[tipo] = None
        raw_events[tipo].clear()

    log_status(f"Coletando RAW {tipo} ({descricao})...")
    if not enviar_comando(f"RAW_{tipo}"):
        with raw_lock:
            raw_busy[tipo] = False
        return

    threading.Thread(
        target=_aguardar_raw,
        args=(tipo, descricao, callback),
        daemon=True
    ).start()

def tare_ambas():
    if enviar_comando("TARE"):
        log_status("Tara enviada para ambas as células.")

def tare_empuxo():
    if enviar_comando("TARE_E"):
        log_status("Tara enviada para empuxo.")

def tare_torque():
    if enviar_comando("TARE_T"):
        log_status("Tara enviada para torque.")

def coletar_raw_empuxo():
    solicitar_raw("E", "vazio", _raw_vazio_recebido)

def coletar_raw_torque():
    solicitar_raw("T", "vazio", _raw_vazio_recebido)

def _aguardar_raw(tipo, descricao, callback):
    recebeu = raw_events[tipo].wait(RAW_TIMEOUT_S)
    with raw_lock:
        valor = raw_values[tipo]
        raw_busy[tipo] = False

    if recebeu and valor is not None:
        ui_call(callback, tipo, valor)
    else:
        ui_call(log_status, f"Timeout aguardando RAW {tipo} ({descricao}).")

def _raw_vazio_recebido(tipo, valor):
    raw_vazio[tipo] = valor
    limpar_pontos_calibracao(tipo)
    if tipo == "E":
        lbl_raw_e.config(text=f"RAW vazio: {valor:.0f}")
        log_status(f"RAW empuxo (vazio): {valor:.0f}")
    else:
        lbl_raw_t.config(text=f"RAW vazio: {valor:.0f}")
        log_status(f"RAW torque (vazio): {valor:.0f}")

def nome_tipo_calibracao(tipo):
    return "empuxo" if tipo == "E" else "torque"

def obter_labels_pontos_calibracao(tipo):
    try:
        return lbl_pontos_e if tipo == "E" else lbl_pontos_t
    except NameError:
        return []

def obter_label_resultado_calibracao(tipo):
    try:
        return lbl_fator_e if tipo == "E" else lbl_fator_t
    except NameError:
        return None

def atualizar_label_ponto_calibracao(tipo, indice):
    labels = obter_labels_pontos_calibracao(tipo)
    if indice >= len(labels):
        return

    ponto = calib_pontos[tipo][indice]
    if ponto["raw"] is None or ponto["forca"] is None:
        labels[indice].config(text=f"P{indice + 1}: RAW --")
    else:
        labels[indice].config(
            text=f"P{indice + 1}: RAW {ponto['raw']:.0f} / {ponto['forca']:.3f} N"
        )

def atualizar_labels_pontos_calibracao(tipo):
    for indice in range(3):
        atualizar_label_ponto_calibracao(tipo, indice)

def limpar_pontos_calibracao(tipo):
    for ponto in calib_pontos[tipo]:
        ponto["forca"] = None
        ponto["raw"] = None
    atualizar_labels_pontos_calibracao(tipo)
    lbl_resultado = obter_label_resultado_calibracao(tipo)
    if lbl_resultado is not None:
        lbl_resultado.config(text="Calibracao: --")

def ler_forca_referencia(entry_ref):
    try:
        forca_ref = float(entry_ref.get().replace(",", "."))
        if forca_ref == 0:
            messagebox.showerror("Erro", "A força de referência não pode ser zero.")
            return None
        return forca_ref
    except ValueError:
        messagebox.showerror("Erro", "Digite uma força de referência numérica válida em N.")
        return None

def coletar_ponto_calibracao(tipo, indice, entry_ref):
    forca_ref = ler_forca_referencia(entry_ref)
    if forca_ref is None:
        return

    raw_zero = raw_vazio[tipo]
    if raw_zero is None:
        messagebox.showwarning("Aviso", "Colete o RAW vazio primeiro.")
        return

    def ao_receber_raw_ponto(tipo_recebido, raw_ponto):
        ponto = calib_pontos[tipo_recebido][indice]
        ponto["forca"] = forca_ref
        ponto["raw"] = raw_ponto
        atualizar_label_ponto_calibracao(tipo_recebido, indice)
        nome = nome_tipo_calibracao(tipo_recebido)
        log_status(
            f"Ponto {indice + 1} {nome}: RAW {raw_ponto:.0f}, "
            f"forca {forca_ref:.3f} N"
        )

    solicitar_raw(tipo, f"ponto {indice + 1} ({forca_ref:.3f} N)", ao_receber_raw_ponto)

def aplicar_calibracao_3p(tipo, lbl_resultado):
    raw_zero = raw_vazio[tipo]
    if raw_zero is None:
        messagebox.showwarning("Aviso", "Colete o RAW vazio primeiro.")
        return

    pontos = calib_pontos[tipo]
    if any(ponto["raw"] is None or ponto["forca"] is None for ponto in pontos):
        messagebox.showwarning("Aviso", "Colete os 3 pontos com pesos diferentes antes de aplicar.")
        return

    forcas = [ponto["forca"] for ponto in pontos]
    raws = [ponto["raw"] for ponto in pontos]
    if len({round(forca, 6) for forca in forcas}) != 3:
        messagebox.showerror("Erro", "Use 3 pesos/forcas diferentes.")
        return

    deltas = [raw - raw_zero for raw in raws]
    if any(delta == 0 for delta in deltas):
        messagebox.showerror("Erro", "Um ponto ficou igual ao RAW vazio. Use um peso maior ou refaca a coleta.")
        return

    if len({int(round(delta)) for delta in deltas}) != 3:
        messagebox.showerror("Erro", "Os 3 pontos RAW precisam ser diferentes.")
        return

    raw_zero_int = int(round(raw_zero))
    campos = [str(raw_zero_int)]
    for raw, forca in zip(raws, forcas):
        campos.append(str(int(round(raw))))
        campos.append(f"{forca:.6f}")

    cmd = f"CAL3_{tipo}:" + ":".join(campos)
    if not enviar_comando(cmd):
        return

    nome = nome_tipo_calibracao(tipo)
    lbl_resultado.config(text="Calibracao 3 pontos aplicada")
    detalhes = ", ".join(
        f"P{i + 1}: raw {raws[i]:.0f} / {forcas[i]:.3f} N"
        for i in range(3)
    )
    log_status(f"Calibracao 3 pontos {nome} enviada. Zero: {raw_zero_int}. {detalhes}")

def aplicar_calibracao_empuxo():
    aplicar_calibracao_3p("E", lbl_fator_e)

def aplicar_calibracao_torque():
    aplicar_calibracao_3p("T", lbl_fator_t)

# ============================================================
#  EXPORTAÇÃO CSV
# ============================================================
def iniciar_gravacao():
    global gravando, gravacao_inicio_local
    if not conectado or not ser or not ser.is_open:
        messagebox.showwarning("Aviso", "Conecte à porta serial antes de iniciar a gravação.")
        return
    if gravando:
        return

    gravando = True
    gravacao_inicio_local = time.monotonic()
    try:
        btn_start_gravacao.config(state="disabled")
        btn_stop_gravacao.config(state="normal")
        lbl_gravacao_status.config(text="Gravando", fg=GREEN)
    except Exception:
        pass
    log_status("Gravação iniciada.")

def parar_gravacao():
    global gravando, gravacao_inicio_local, gravacao_duracao_acumulada
    if not gravando:
        return

    gravacao_duracao_acumulada = tempo_gravacao_atual()
    gravacao_inicio_local = None
    gravando = False
    try:
        btn_start_gravacao.config(state="normal")
        btn_stop_gravacao.config(state="disabled")
        lbl_gravacao_status.config(text="Parada", fg=MUTED_CLR)
    except Exception:
        pass
    log_status("Gravação parada.")

def resetar_gravacao():
    global gravando, gravacao_inicio_local, gravacao_duracao_acumulada
    gravando = False
    gravacao_inicio_local = None
    gravacao_duracao_acumulada = 0.0
    try:
        btn_start_gravacao.config(state="normal")
        btn_stop_gravacao.config(state="disabled")
        lbl_gravacao_status.config(text="Parada", fg=MUTED_CLR)
        lbl_cronometro.config(text="00:00:00")
    except Exception:
        pass

def ler_entry(entry, padrao):
    try:
        valor = entry.get().strip()
        return valor if valor else padrao
    except Exception:
        return padrao

def atualizar_densidade_ar_local(*args):
    global densidade_ar_local
    try:
        valor_txt = var_densidade_ar.get().strip().replace(",", ".")
        valor = float(valor_txt)
        if valor <= 0:
            raise ValueError
        densidade_ar_local = valor
        try:
            lbl_fator_densidade.config(text=f"Fator: {densidade_ar_local / DENSIDADE_AR_PADRAO:.4f}")
        except Exception:
            pass
    except Exception:
        try:
            lbl_fator_densidade.config(text="Valor inválido")
        except Exception:
            pass

def salvar_graficos_png(caminho_png):
    tempos = [p[0] for p in historico]
    empuxos = [p[1] for p in historico]
    torques = [p[2] for p in historico]
    x_max = max(tempos) if tempos else 1.0
    if x_max <= 0:
        x_max = 1.0

    titulo_teste = ler_entry(entry_titulo_teste, "Teste sem título")
    motor = ler_entry(entry_motor, "Motor não informado")
    helice = ler_entry(entry_helice, "Hélice não informada")
    local = ler_entry(entry_local_teste, "Local não informado")
    temperatura = ler_entry(entry_temp_local, "não informada")
    if temperatura != "não informada" and "°" not in temperatura:
        temperatura = f"{temperatura} °C"

    agora = datetime.now()
    data_txt = agora.strftime("%d/%m/%Y")
    hora_txt = agora.strftime("%H:%M:%S")

    fig_png, (ax_png_emp, ax_png_tor) = plt.subplots(2, 1, figsize=(10, 7.5), sharex=True)
    fig_png.patch.set_facecolor("white")
    fig_png.suptitle(
        f"{titulo_teste}\nMotor: {motor} | Hélice: {helice}",
        fontsize=14,
        fontweight="bold",
        y=0.97
    )

    ax_png_emp.plot(tempos, empuxos, color=ACCENT, linewidth=1.8)
    ax_png_emp.set_title("Empuxo x Tempo")
    ax_png_emp.set_ylabel(f"Empuxo ({EMPUXO_UNIDADE})")
    ax_png_emp.set_xlim(0, x_max)
    ax_png_emp.set_ylim(EMPUXO_Y_MIN, EMPUXO_Y_MAX)
    ax_png_emp.grid(True, linestyle="--", alpha=0.35)

    ax_png_tor.plot(tempos, torques, color=TORQUE_COR, linewidth=1.8)
    ax_png_tor.set_title("Torque x Tempo")
    ax_png_tor.set_xlabel("Tempo (s)")
    ax_png_tor.set_ylabel(f"Torque ({TORQUE_UNIDADE})")
    ax_png_tor.set_xlim(0, x_max)
    ax_png_tor.set_ylim(TORQUE_Y_MIN, TORQUE_Y_MAX)
    ax_png_tor.grid(True, linestyle="--", alpha=0.35)

    fig_png.text(
        0.01,
        0.015,
        f"Local: {local} | Temperatura local: {temperatura} | Densidade do ar: {densidade_ar_local:.4f} kg/m3 | Data: {data_txt} | Horário: {hora_txt}",
        fontsize=9,
        color="#222222"
    )
    fig_png.tight_layout(rect=[0, 0.05, 1, 0.93])
    fig_png.savefig(caminho_png, dpi=150)
    plt.close(fig_png)

def exportar_csv():
    if gravando:
        messagebox.showwarning("Aviso", "Pare a gravação antes de exportar o CSV.")
        return
    if not historico:
        messagebox.showwarning("Aviso", "Nenhum dado para exportar.")
        return
    nome_default = f"ensaio_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    caminho = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[("CSV", "*.csv"), ("Todos", "*.*")],
        initialfile=nome_default,
        title="Salvar ensaio como"
    )
    if not caminho:
        return
    try:
        with open(caminho, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["tempo_s", "empuxo_N", "torque_N_m"])
            writer.writerows(historico)
        caminho_png = os.path.splitext(caminho)[0] + "_graficos.png"
        salvar_graficos_png(caminho_png)
        log_status(f"Exportado: {os.path.basename(caminho)}")
        log_status(f"Gráficos exportados: {os.path.basename(caminho_png)}")
        messagebox.showinfo("Sucesso", f"Arquivos salvos:\n{caminho}\n{caminho_png}")
    except Exception as e:
        messagebox.showerror("Erro", f"Falha ao salvar:\n{e}")

def limpar_dados():
    if messagebox.askyesno("Confirmar", "Limpar todos os dados do ensaio atual?"):
        limpar_dados_locais()
        log_status("Dados limpos.")

def limpar_dados_locais():
    global leitura_raw_e, leitura_raw_t, tempo_grafico_offset, ultimo_tempo_serial
    leitura_raw_e = None
    leitura_raw_t = None
    tempo_grafico_offset = 0.0
    ultimo_tempo_serial = None
    raw_vazio["E"] = None
    raw_vazio["T"] = None
    for tipo in calib_pontos:
        for ponto in calib_pontos[tipo]:
            ponto["forca"] = None
            ponto["raw"] = None
    cancelar_raw_pendentes()
    resetar_gravacao()
    tempo_buf.clear()
    empuxo_buf.clear()
    torque_buf.clear()
    historico.clear()
    try:
        lbl_empuxo_val.config(text="—")
        lbl_torque_val.config(text="—")
        lbl_raw_e.config(text="RAW vazio: —")
        lbl_raw_t.config(text="RAW vazio: —")
        lbl_fator_e.config(text="Calibracao: --")
        lbl_fator_t.config(text="Calibracao: --")
        atualizar_labels_pontos_calibracao("E")
        atualizar_labels_pontos_calibracao("T")
        linha_empuxo.set_data([], [])
        linha_torque.set_data([], [])
        txt_emp.set_text("")
        txt_tor.set_text("")
        canvas.draw_idle()
    except Exception:
        pass

def resetar_programa():
    if not messagebox.askyesno(
        "Confirmar reset",
        "Resetar a interface e reiniciar o ESP32 conectado?"
    ):
        return

    limpar_dados_locais()
    if conectado and ser and ser.is_open:
        if enviar_comando("RESET"):
            log_status("Reset enviado ao ESP32.")
    else:
        log_status("Interface resetada localmente.")

# ============================================================
#  JANELA PRINCIPAL
# ============================================================
root = tk.Tk()
root.title("Dinamômetro — Interface de Ensaio")
root.configure(bg="#666666")
root.geometry("1300x800")
root.minsize(1100, 700)
var_densidade_ar = tk.StringVar(value=f"{DENSIDADE_AR_PADRAO:.3f}")
var_titulo_teste = tk.StringVar()
var_motor = tk.StringVar()
var_helice = tk.StringVar()

DARK_BG    = "#666666"
PANEL_BG   = "#303030"
CARD_BG    = "#333333"
BORDER_CLR = "#8a8a8a"
FIELD_BG   = "#f2f2f2"
FIELD_FG   = "#202020"
ACCENT     = "#ffc400"
TEXT_CLR   = "#ffffff"
MUTED_CLR  = "#b8b8b8"
GREEN      = "#078a34"
RED_CLR    = "#8f3434"
BLUE       = "#4b5068"

style = ttk.Style()
style.theme_use("clam")
style.configure("TNotebook",        background=DARK_BG, borderwidth=0)
style.configure("TNotebook.Tab",    background=PANEL_BG, foreground=TEXT_CLR,
                padding=[14, 6], font=("Segoe UI", 9, "bold"))
style.map("TNotebook.Tab",          background=[("selected", CARD_BG)])
style.configure("TFrame",           background=DARK_BG)
style.configure("TLabel",           background=DARK_BG, foreground=TEXT_CLR,
                font=("Segoe UI", 9))
style.configure("TEntry",           fieldbackground=FIELD_BG, foreground=FIELD_FG,
                insertcolor=FIELD_FG)
style.configure("TCombobox",        fieldbackground=FIELD_BG, foreground=FIELD_FG,
                selectbackground=CARD_BG)

def make_btn(parent, text, cmd, color=CARD_BG, fg=TEXT_CLR, **kw):
    return tk.Button(parent, text=text, command=cmd,
                     bg=color, fg=fg, activebackground="#4a4a4a",
                     activeforeground="white", relief="solid", bd=2,
                     highlightbackground=BORDER_CLR, highlightcolor=BORDER_CLR,
                     font=("Segoe UI", 9, "bold"), cursor="hand2",
                     padx=10, pady=4, **kw)

def carregar_logo():
    try:
        original = tk.PhotoImage(file=LOGO_PATH)
        escala = max(
            1,
            int(max(
                original.width() / LOGO_MAX_WIDTH,
                original.height() / LOGO_MAX_HEIGHT
            ) + 0.999)
        )
        return original, original.subsample(escala, escala)
    except Exception:
        return None, None

# ============================================================
#  BARRA SUPERIOR — Conexão Serial
# ============================================================
frame_top = tk.Frame(root, bg=PANEL_BG, pady=6)
frame_top.pack(fill="x", padx=8, pady=(8, 0))
frame_top.columnconfigure(8, weight=1)

tk.Label(frame_top, text="Conexão Serial", bg=PANEL_BG, fg=ACCENT,
         font=("Segoe UI", 9, "bold")).grid(row=0, column=0, columnspan=8,
         sticky="w", padx=10)

tk.Label(frame_top, text="Porta:", bg=PANEL_BG, fg=TEXT_CLR,
         font=("Segoe UI", 9)).grid(row=1, column=0, padx=(10, 4), pady=4)

combo_porta = ttk.Combobox(frame_top, values=listar_portas(), width=10)
combo_porta.grid(row=1, column=1, padx=4)
if listar_portas():
    combo_porta.current(0)

def refresh_portas():
    combo_porta["values"] = listar_portas()

make_btn(frame_top, "↻", refresh_portas, color=PANEL_BG).grid(row=1, column=2, padx=2)
btn_conectar    = make_btn(frame_top, "Conectar",    conectar,    color=GREEN,   fg="white")
btn_conectar.grid(row=1, column=3, padx=6)
btn_desconectar = make_btn(frame_top, "Desconectar", desconectar, color=RED_CLR, fg="white",
                           state="disabled")
btn_desconectar.grid(row=1, column=4, padx=2)

lbl_status_conn = tk.Label(frame_top, text="● Desconectado", bg=PANEL_BG,
                            fg=RED_CLR, font=("Segoe UI", 9, "bold"))
lbl_status_conn.grid(row=1, column=5, padx=20)

make_btn(frame_top, "Reset", resetar_programa, color=CARD_BG, fg="white").grid(row=1, column=6, padx=6)
make_btn(frame_top, "Zerar tempo gráfico", zerar_tempo_grafico, color=BLUE, fg="white").grid(row=1, column=7, padx=6)

logo_original, logo_img = carregar_logo()
if logo_img is not None:
    tk.Label(frame_top, image=logo_img, bg=PANEL_BG).grid(
        row=0, column=9, rowspan=2, sticky="e", padx=(20, 10)
    )

# ============================================================
#  LEITURAS INSTANTÂNEAS
# ============================================================
frame_inst = tk.Frame(root, bg=PANEL_BG, pady=8)
frame_inst.pack(fill="x", padx=8, pady=(6, 0))

tk.Label(frame_inst, text="Leituras Instantâneas", bg=PANEL_BG, fg=ACCENT,
         font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=10)

frame_cards = tk.Frame(frame_inst, bg=PANEL_BG)
frame_cards.pack(fill="x", padx=10, pady=4)

def make_card(parent, titulo, unidade, col):
    f = tk.Frame(parent, bg=CARD_BG, padx=20, pady=10)
    f.grid(row=0, column=col, padx=8, sticky="ew")
    parent.columnconfigure(col, weight=1)
    tk.Label(f, text=titulo, bg=CARD_BG, fg=MUTED_CLR,
             font=("Segoe UI", 8)).pack()
    val = tk.Label(f, text="—", bg=CARD_BG, fg=TEXT_CLR,
                   font=("Segoe UI", 22, "bold"))
    val.pack()
    tk.Label(f, text=unidade, bg=CARD_BG, fg=MUTED_CLR,
             font=("Segoe UI", 8)).pack()
    return val

lbl_empuxo_val = make_card(frame_cards, "EMPUXO",  EMPUXO_UNIDADE, 0)
lbl_torque_val = make_card(frame_cards, "TORQUE",  TORQUE_UNIDADE, 1)

frame_densidade = tk.Frame(frame_inst, bg=PANEL_BG)
frame_densidade.pack(fill="x", padx=18, pady=(0, 4))
tk.Label(frame_densidade, text="Densidade do ar local (kg/m3):", bg=PANEL_BG, fg=TEXT_CLR).pack(side="left")
entry_densidade_ar = ttk.Entry(frame_densidade, width=10, textvariable=var_densidade_ar)
entry_densidade_ar.pack(side="left", padx=(6, 10))
lbl_fator_densidade = tk.Label(
    frame_densidade,
    text=f"Fator: {densidade_ar_local / DENSIDADE_AR_PADRAO:.4f}",
    bg=PANEL_BG,
    fg=MUTED_CLR
)
lbl_fator_densidade.pack(side="left")
var_densidade_ar.trace_add("write", atualizar_densidade_ar_local)

# ============================================================
#  NOTEBOOK — abas
# ============================================================
notebook = ttk.Notebook(root)
notebook.pack(fill="both", expand=True, padx=8, pady=8)

# ============================================================
#  ABA 1 — GRÁFICOS
# ============================================================
aba_graficos = tk.Frame(notebook, bg=DARK_BG)
notebook.add(aba_graficos, text="  📈  Gráficos  ")

frame_info_graficos = tk.Frame(aba_graficos, bg=PANEL_BG, padx=16, pady=8)
frame_info_graficos.pack(fill="x", padx=6, pady=(6, 0))

lbl_titulo_graficos = tk.Label(
    frame_info_graficos,
    text="Teste sem título",
    bg=PANEL_BG,
    fg=ACCENT,
    font=("Segoe UI", 12, "bold")
)
lbl_titulo_graficos.pack(anchor="w")

lbl_info_graficos = tk.Label(
    frame_info_graficos,
    text="Motor: não informado | Hélice: não informada",
    bg=PANEL_BG,
    fg=TEXT_CLR,
    font=("Segoe UI", 9)
)
lbl_info_graficos.pack(anchor="w", pady=(2, 0))

def atualizar_cabecalho_graficos(*args):
    titulo = var_titulo_teste.get().strip() or "Teste sem título"
    motor = var_motor.get().strip() or "não informado"
    helice = var_helice.get().strip() or "não informada"
    lbl_titulo_graficos.config(text=titulo)
    lbl_info_graficos.config(text=f"Motor: {motor} | Hélice: {helice}")

for var_meta_grafico in (var_titulo_teste, var_motor, var_helice):
    var_meta_grafico.trace_add("write", atualizar_cabecalho_graficos)

fig, (ax_emp, ax_tor) = plt.subplots(1, 2, figsize=(12, 4))
fig.patch.set_facecolor(DARK_BG)

for ax, titulo, cor, ylabel in [
    (ax_emp, "Empuxo × Tempo", ACCENT,   f"Empuxo ({EMPUXO_UNIDADE})"),
    (ax_tor, "Torque × Tempo",  TORQUE_COR,    f"Torque ({TORQUE_UNIDADE})"),
]:
    ax.set_facecolor(PANEL_BG)
    ax.set_title(titulo, color=TEXT_CLR, fontsize=10, fontweight="bold")
    ax.set_xlabel("Tempo (s)", color=MUTED_CLR, fontsize=8)
    ax.set_ylabel(ylabel, color=MUTED_CLR, fontsize=8)
    ax.tick_params(colors=MUTED_CLR, labelsize=7)
    ax.grid(True, linestyle="--", alpha=0.25, color=BORDER_CLR)
    for sp in ax.spines.values():
        sp.set_edgecolor(BORDER_CLR)

ax_emp.set_ylim(EMPUXO_Y_MIN, EMPUXO_Y_MAX)
ax_tor.set_ylim(TORQUE_Y_MIN, TORQUE_Y_MAX)

linha_empuxo, = ax_emp.plot([], [], color=ACCENT, linewidth=1.8)
linha_torque, = ax_tor.plot([], [], color=TORQUE_COR,   linewidth=1.8)

txt_emp = ax_emp.text(0.99, 0.95, "", transform=ax_emp.transAxes,
                      ha="right", va="top", color=TEXT_CLR, fontsize=10,
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="#242424", alpha=0.88))
txt_tor = ax_tor.text(0.99, 0.95, "", transform=ax_tor.transAxes,
                      ha="right", va="top", color=TEXT_CLR, fontsize=10,
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="#242424", alpha=0.88))

plt.tight_layout(pad=2.0)

canvas = FigureCanvasTkAgg(fig, master=aba_graficos)
canvas.get_tk_widget().pack(fill="both", expand=True, padx=6, pady=6)

def atualizar_grafico(frame):
    if len(tempo_buf) < 2:
        return linha_empuxo, linha_torque

    t  = list(tempo_buf)
    e  = list(empuxo_buf)
    tr = list(torque_buf)

    linha_empuxo.set_data(t, e)
    linha_torque.set_data(t, tr)

    span = max(t[-1] - t[0], 0.1)
    x_min = max(0.0, t[0])
    x_max = t[-1] + span * 0.02 + 0.1
    ax_emp.set_xlim(x_min, x_max)
    ax_tor.set_xlim(x_min, x_max)
    ax_emp.set_ylim(EMPUXO_Y_MIN, EMPUXO_Y_MAX)
    ax_tor.set_ylim(TORQUE_Y_MIN, TORQUE_Y_MAX)

    txt_emp.set_text(f"{e[-1]:.3f} {EMPUXO_UNIDADE}")
    txt_tor.set_text(f"{tr[-1]:.3f} {TORQUE_UNIDADE}")

    canvas.draw_idle()
    return linha_empuxo, linha_torque

ani = animation.FuncAnimation(fig, atualizar_grafico,
                               interval=INTERVALO_MS, blit=False,
                               cache_frame_data=False)

# ============================================================
#  ABA 2 — CALIBRAÇÃO
# ============================================================
aba_cal = tk.Frame(notebook, bg=DARK_BG)
notebook.add(aba_cal, text="  ⚖️  Calibração  ")

canvas_cal = tk.Canvas(aba_cal, bg=DARK_BG, highlightthickness=0)
scroll_cal = ttk.Scrollbar(aba_cal, orient="vertical", command=canvas_cal.yview)
frm_cal_conteudo = tk.Frame(canvas_cal, bg=DARK_BG)
janela_cal = canvas_cal.create_window((0, 0), window=frm_cal_conteudo, anchor="nw")

canvas_cal.configure(yscrollcommand=scroll_cal.set)
canvas_cal.pack(side="left", fill="both", expand=True)
scroll_cal.pack(side="right", fill="y")

def atualizar_area_scroll_cal(event):
    canvas_cal.configure(scrollregion=canvas_cal.bbox("all"))

def ajustar_largura_scroll_cal(event):
    canvas_cal.itemconfigure(janela_cal, width=event.width)

def rolar_calibracao_mouse(event):
    if notebook.select() == str(aba_cal):
        canvas_cal.yview_scroll(int(-1 * (event.delta / 120)), "units")

frm_cal_conteudo.bind("<Configure>", atualizar_area_scroll_cal)
canvas_cal.bind("<Configure>", ajustar_largura_scroll_cal)
root.bind_all("<MouseWheel>", rolar_calibracao_mouse, add="+")

def secao(parent, titulo):
    f = tk.LabelFrame(parent, text=f"  {titulo}  ", bg=PANEL_BG,
                      fg=ACCENT, font=("Segoe UI", 9, "bold"),
                      bd=1, relief="groove", padx=14, pady=10)
    f.pack(fill="x", padx=20, pady=10)
    return f

def criar_linhas_calibracao(parent, tipo, padroes):
    entries = []
    labels = []
    for indice, padrao in enumerate(padroes):
        row = 2 + indice
        tk.Label(parent, text=f"Ponto {indice + 1} - forca (N):",
                 bg=PANEL_BG, fg=TEXT_CLR).grid(row=row, column=0, sticky="w", padx=(0,4), pady=4)
        entry = ttk.Entry(parent, width=10)
        entry.insert(0, padrao)
        entry.grid(row=row, column=1, sticky="w", padx=(0,8), pady=4)
        make_btn(
            parent,
            f"Coletar P{indice + 1}",
            lambda i=indice, e=entry: coletar_ponto_calibracao(tipo, i, e),
            color=BLUE
        ).grid(row=row, column=2, sticky="w", padx=6, pady=4)
        lbl = tk.Label(parent, text=f"P{indice + 1}: RAW --",
                       bg=PANEL_BG, fg=TEXT_CLR)
        lbl.grid(row=row, column=3, sticky="w", padx=10, pady=4)
        entries.append(entry)
        labels.append(lbl)
    parent.columnconfigure(3, weight=1)
    return entries, labels

# --- Tara ---
frm_tare = secao(frm_cal_conteudo, "Tara")
tk.Label(frm_tare, text="Zera as células antes de iniciar o ensaio.",
         bg=PANEL_BG, fg=MUTED_CLR).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0,8))
make_btn(frm_tare, "Tara — Empuxo",    tare_empuxo, color=BLUE).grid(row=1, column=0, padx=6)
make_btn(frm_tare, "Tara — Torque",    tare_torque, color=BLUE).grid(row=1, column=1, padx=6)
make_btn(frm_tare, "Tara — Ambas",     tare_ambas,  color=CARD_BG, fg="white").grid(row=1, column=2, padx=6)

# --- Calibração Empuxo ---
frm_cal_e = secao(frm_cal_conteudo, "Calibração — Empuxo")
tk.Label(frm_cal_e, text="1. Retire qualquer peso da celula e colete o RAW vazio.\n"
         "2. Aplique 3 pesos/forcas diferentes, colete P1, P2 e P3, depois aplique a calibracao.",
         bg=PANEL_BG, fg=MUTED_CLR, justify="left").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0,8))

make_btn(frm_cal_e, "1. Coletar RAW vazio", coletar_raw_empuxo, color=BLUE).grid(row=1, column=0, padx=6)
lbl_raw_e = tk.Label(frm_cal_e, text="RAW vazio: —", bg=PANEL_BG, fg=TEXT_CLR)
lbl_raw_e.grid(row=1, column=1, columnspan=3, sticky="w", padx=10)

entries_forca_e, lbl_pontos_e = criar_linhas_calibracao(frm_cal_e, "E", ("9.807", "19.614", "29.421"))

make_btn(frm_cal_e, "Aplicar calibracao 3 pontos", aplicar_calibracao_empuxo, color=GREEN, fg="white").grid(row=5, column=0, columnspan=2, sticky="w", padx=6, pady=(8,0))
lbl_fator_e = tk.Label(frm_cal_e, text="Calibracao: --", bg=PANEL_BG, fg=GREEN,
                        font=("Segoe UI", 9, "bold"))
lbl_fator_e.grid(row=5, column=2, columnspan=2, sticky="w", padx=10, pady=(8,0))

# --- Calibração Torque ---
frm_cal_t = secao(frm_cal_conteudo, "Calibração — Torque")
tk.Label(frm_cal_t, text="1. Retire qualquer peso da celula e colete o RAW vazio.\n"
         "2. Aplique 3 pesos/forcas diferentes, colete P1, P2 e P3, depois aplique a calibracao.",
         bg=PANEL_BG, fg=MUTED_CLR, justify="left").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0,8))

make_btn(frm_cal_t, "1. Coletar RAW vazio", coletar_raw_torque, color=BLUE).grid(row=1, column=0, padx=6)
lbl_raw_t = tk.Label(frm_cal_t, text="RAW vazio: —", bg=PANEL_BG, fg=TEXT_CLR)
lbl_raw_t.grid(row=1, column=1, columnspan=3, sticky="w", padx=10)

entries_forca_t, lbl_pontos_t = criar_linhas_calibracao(frm_cal_t, "T", ("9.807", "19.614", "29.421"))

make_btn(frm_cal_t, "Aplicar calibracao 3 pontos", aplicar_calibracao_torque, color=GREEN, fg="white").grid(row=5, column=0, columnspan=2, sticky="w", padx=6, pady=(8,0))
lbl_fator_t = tk.Label(frm_cal_t, text="Calibracao: --", bg=PANEL_BG, fg=GREEN,
                        font=("Segoe UI", 9, "bold"))
lbl_fator_t.grid(row=5, column=2, columnspan=2, sticky="w", padx=10, pady=(8,0))

# ============================================================
#  ABA 3 — EXPORTAR
# ============================================================
aba_export = tk.Frame(notebook, bg=DARK_BG)
notebook.add(aba_export, text="  💾  Exportar  ")

frm_exp = tk.Frame(aba_export, bg=PANEL_BG, padx=30, pady=30)
frm_exp.pack(fill="x", padx=20, pady=20)

tk.Label(frm_exp, text="Exportar dados do ensaio", bg=PANEL_BG, fg=ACCENT,
         font=("Segoe UI", 12, "bold")).pack(anchor="w")
tk.Label(frm_exp, text="Salva em CSV apenas os pontos capturados entre Start e Stop.\n"
         "Colunas: tempo_s | empuxo_N | torque_N_m",
         bg=PANEL_BG, fg=MUTED_CLR, justify="left").pack(anchor="w", pady=(6, 20))

frm_meta = tk.LabelFrame(frm_exp, text="  Identificação do ensaio  ", bg=PANEL_BG,
                         fg=ACCENT, font=("Segoe UI", 9, "bold"),
                         bd=2, relief="groove", padx=12, pady=10)
frm_meta.pack(fill="x", pady=(0, 16))

tk.Label(frm_meta, text="Título do teste:", bg=PANEL_BG, fg=TEXT_CLR).grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
entry_titulo_teste = ttk.Entry(frm_meta, width=64, textvariable=var_titulo_teste)
entry_titulo_teste.grid(row=0, column=1, columnspan=3, sticky="ew", padx=(0, 16), pady=4)

tk.Label(frm_meta, text="Motor:", bg=PANEL_BG, fg=TEXT_CLR).grid(row=1, column=0, sticky="w", padx=(0, 6), pady=4)
entry_motor = ttk.Entry(frm_meta, width=28, textvariable=var_motor)
entry_motor.grid(row=1, column=1, sticky="ew", padx=(0, 16), pady=4)

tk.Label(frm_meta, text="Hélice:", bg=PANEL_BG, fg=TEXT_CLR).grid(row=1, column=2, sticky="w", padx=(0, 6), pady=4)
entry_helice = ttk.Entry(frm_meta, width=28, textvariable=var_helice)
entry_helice.grid(row=1, column=3, sticky="ew", padx=(0, 16), pady=4)

tk.Label(frm_meta, text="Local:", bg=PANEL_BG, fg=TEXT_CLR).grid(row=2, column=0, sticky="w", padx=(0, 6), pady=4)
entry_local_teste = ttk.Entry(frm_meta, width=28)
entry_local_teste.grid(row=2, column=1, sticky="ew", padx=(0, 16), pady=4)

tk.Label(frm_meta, text="Temperatura local:", bg=PANEL_BG, fg=TEXT_CLR).grid(row=2, column=2, sticky="w", padx=(0, 6), pady=4)
entry_temp_local = ttk.Entry(frm_meta, width=12)
entry_temp_local.grid(row=2, column=3, sticky="w", padx=(0, 16), pady=4)

frm_meta.columnconfigure(1, weight=1)
frm_meta.columnconfigure(3, weight=1)

frame_gravacao = tk.Frame(frm_exp, bg=PANEL_BG)
frame_gravacao.pack(anchor="w", fill="x", pady=(0, 16))

btn_start_gravacao = make_btn(frame_gravacao, "Start", iniciar_gravacao, color=GREEN, fg="white")
btn_start_gravacao.pack(side="left", padx=(0, 8))

btn_stop_gravacao = make_btn(frame_gravacao, "Stop", parar_gravacao, color=RED_CLR, fg="white", state="disabled")
btn_stop_gravacao.pack(side="left", padx=(0, 18))

tk.Label(frame_gravacao, text="Tempo:", bg=PANEL_BG, fg=MUTED_CLR).pack(side="left")
lbl_cronometro = tk.Label(frame_gravacao, text="00:00:00", bg=PANEL_BG, fg=TEXT_CLR,
                          font=("Consolas", 18, "bold"))
lbl_cronometro.pack(side="left", padx=(8, 18))

lbl_gravacao_status = tk.Label(frame_gravacao, text="Parada", bg=PANEL_BG, fg=MUTED_CLR,
                               font=("Segoe UI", 9, "bold"))
lbl_gravacao_status.pack(side="left")

frame_btns_exp = tk.Frame(frm_exp, bg=PANEL_BG)
frame_btns_exp.pack(anchor="w")

make_btn(frame_btns_exp, "💾  Salvar CSV + PNG...", exportar_csv, color=GREEN, fg="white").pack(side="left", padx=(0,12))
make_btn(frame_btns_exp, "🗑  Limpar dados", limpar_dados, color=RED_CLR, fg="white").pack(side="left")

tk.Label(frm_exp, text="", bg=PANEL_BG).pack(pady=10)
tk.Label(frm_exp, text="Pontos coletados nesta sessão:", bg=PANEL_BG, fg=MUTED_CLR).pack(anchor="w")

def atualizar_contador():
    lbl_contador.config(text=str(len(historico)))
    lbl_cronometro.config(text=formatar_tempo(tempo_gravacao_atual()))
    root.after(250, atualizar_contador)

lbl_contador = tk.Label(frm_exp, text="0", bg=PANEL_BG, fg=TEXT_CLR,
                         font=("Segoe UI", 28, "bold"))
lbl_contador.pack(anchor="w")
atualizar_contador()

# ============================================================
#  LOG DE STATUS (rodapé)
# ============================================================
frame_log = tk.Frame(root, bg=PANEL_BG)
frame_log.pack(fill="x", padx=8, pady=(0, 6))

txt_log = tk.Text(frame_log, height=3, bg="#242424", fg=MUTED_CLR,
                  font=("Consolas", 8), state="disabled", relief="flat",
                  insertbackground=TEXT_CLR)
txt_log.pack(fill="x", padx=6, pady=4)

log_status("Interface iniciada. Selecione a porta e clique em Conectar.")

# ============================================================
#  ENCERRAMENTO
# ============================================================
def on_close():
    global lendo
    if gravando:
        parar_gravacao()
    lendo = False
    cancelar_raw_pendentes()
    time.sleep(0.2)
    if ser and ser.is_open:
        ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
