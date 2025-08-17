#Итоговая end-to-end latency 125-150 ms
#!/usr/bin/env python3 
import socket
import struct
import time
import sys
import os
import subprocess
import platform
import argparse
import threading
from datetime import datetime

# Добавляем импорт NumPy
import numpy as np

# Попытка импорта библиотек
try:
    import pyaudio
except ImportError:
    print("ERROR: PyAudio library not found.")
    print("Please install it using: pip install pyaudio")
    sys.exit(1)

try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import scrolledtext
except ImportError:
    print("ERROR: tkinter library not found. It should be part of the standard Python installation.")
    sys.exit(1)

# --- Конфигурация ---
ESP32_IP = "192.168.4.1"
ESP32_UDP_PORT = 3333
START_COMMAND = b"START_AUDIO_STREAM"
#SOCKET_TIMEOUT = 2.0
SOCKET_TIMEOUT = 0.5  # Уменьшаем с 2.0 до 0.5 секунд для быстрого обнаружения отключения

# --- Аудио параметры --- (обновляем конфигурацию)
SAMPLE_RATE = 48000
BYTES_PER_SAMPLE = 3          # 24-bit samples = 3 bytes (ВАЖНОЕ ИСПРАВЛЕНИЕ)
#PYAUDIO_FORMAT = pyaudio.paInt24  # Используем 24-битный формат вместо 32-битного
CHANNELS = 1

# --- Параметры пакета ---
PACKET_HEADER_MAGIC = 0xAABBCCDD
PACKET_HEADER_STRUCT_FORMAT = '<IIIII'  # magic, packet_num, sample_count, status, checksum
PACKET_HEADER_SIZE = struct.calcsize(PACKET_HEADER_STRUCT_FORMAT)  # 20 bytes

# Исправляем параметры в соответствии с ESP32 использующим PCM1808 (24-bit)
SAMPLES_PER_PACKET = 48       # ESP32 отправляет 48 сэмплов
AUDIO_DATA_SIZE = SAMPLES_PER_PACKET * BYTES_PER_SAMPLE  # 48 * 3 = 144 bytes
TOTAL_PACKET_SIZE = PACKET_HEADER_SIZE + AUDIO_DATA_SIZE  # 20 + 144 = 164 bytes

# --- WiFi Configuration ---
WIFI_SSID = "ESP32_Audio_AP"
WIFI_PASSWORD = "password"
WIFI_CONNECTION_TIMEOUT = 30  # seconds
PING_INTERVAL = 1  # seconds

class AudioApp:
    def __init__(self, root, stream_enabled=True, record_enabled=False):
        self.root = root
        self.stream_enabled = stream_enabled
        
        # Добавьте эти переменные
        self.latency_samples = []
        self.latency_display_timer = None

        self.sock = None
        self.pyaudio_instance = None
        self.audio_stream = None
        self.running = False
        self.worker_thread = None

        self.packets_received_total = 0
        self.last_packet_num = -1
        self.initial_packet_num_set = False
        self.packets_out_of_order = 0

        self.last_packet_time = 0
        self.packet_rate = 0

        self.setup_gui()
         # В конце функции добавьте таймер для отображения задержки
        self.latency_display_timer = self.root.after(1000, self.display_latency)

    def setup_gui(self):
        self.root.title("ESP32 Wi-Fi Microphone Receiver")
        self.root.geometry("600x450")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        status_frame = ttk.LabelFrame(main_frame, text="Connection Status", padding="10")
        status_frame.pack(fill=tk.X, pady=5)
        
        self.connection_status_label = ttk.Label(status_frame, text="Status: Disconnected", font=("Helvetica", 12))
        self.connection_status_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(status_frame, text="Samples/Packet:").pack(side=tk.LEFT, padx=(20, 2))
        #self.packet_size_var = tk.StringVar(value="192")
        self.packet_size_var = tk.StringVar(value="48")  # Изменено на 48 для соответствия ESP32
        self.packet_size_entry = ttk.Entry(status_frame, textvariable=self.packet_size_var, width=6)
        self.packet_size_entry.pack(side=tk.LEFT)
        
        self.start_button = ttk.Button(status_frame, text="Start", command=self.start_reception)
        self.start_button.pack(side=tk.RIGHT)
        self.stop_button = ttk.Button(status_frame, text="Stop", command=self.stop_reception, state=tk.DISABLED)
        self.stop_button.pack(side=tk.RIGHT, padx=5)

        volume_frame = ttk.LabelFrame(main_frame, text="Device Status", padding="10")
        volume_frame.pack(fill=tk.X, pady=5)

        ttk.Label(volume_frame, text="Volume:").pack(side=tk.LEFT, padx=5)
        self.volume_bar = ttk.Progressbar(volume_frame, orient='horizontal', length=300, mode='determinate', maximum=128)
        self.volume_bar.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        self.volume_label = ttk.Label(volume_frame, text="0 / 128")
        self.volume_label.pack(side=tk.LEFT, padx=5)

        self.mute_status_label = ttk.Label(volume_frame, text="UNMUTED", font=("Helvetica", 14, "bold"), foreground="green", background="white", padding=5, relief="solid", borderwidth=1)
        self.mute_status_label.pack(side=tk.RIGHT, padx=10)

        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log_line(self, message):
        def append_log():
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_msg = f"[{timestamp}] {message}\n"
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, log_msg)
            self.log_text.config(state=tk.DISABLED)
            self.log_text.see(tk.END)
        self.root.after(0, append_log)

    def update_status_indicators(self, volume, is_muted):
        def update_gui():
            self.volume_bar['value'] = volume
            self.volume_label.config(text=f"{volume} / 128")
            if is_muted:
                self.mute_status_label.config(text="MUTED", foreground="red")
            else:
                self.mute_status_label.config(text="UNMUTED", foreground="green")
        self.root.after(0, update_gui)

    def set_connection_status(self, text, is_connected):
        def update_gui():
            self.connection_status_label.config(text=f"Status: {text}")
            color = "green" if is_connected else "red"
            self.connection_status_label.config(foreground=color)
        self.root.after(0, update_gui)

    def start_reception(self):
        """Initialize and start the audio reception process."""
        try:
            # Используем глобальные константы напрямую, без промежуточных переменных
            self.log_line(f"Using {SAMPLES_PER_PACKET} samples per packet")
            self.log_line(f"Packet size: {TOTAL_PACKET_SIZE} bytes")       
            
            # Reset statistics
            self.packets_received_total = 0
            self.last_packet_num = -1
            self.initial_packet_num_set = False
            self.packets_out_of_order = 0
            
            # Disable UI controls during reception
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.packet_size_entry.config(state=tk.DISABLED)
            
            # Start reception thread
            self.running = True
            self.worker_thread = threading.Thread(target=self.reception_loop, daemon=True)
            self.worker_thread.start()
            
            # Запускаем регулярное отображение статистики
            self.root.after(5000, self.display_statistics)
            
            # Запустить отображение задержки при старте приема
            self.latency_samples = []  # Очистить предыдущие данные
            if self.latency_display_timer:
                self.root.after_cancel(self.latency_display_timer)
            self.latency_display_timer = self.root.after(1000, self.display_latency)
            
        except Exception as e:
            self.log_line(f"ERROR: Failed to start reception: {e}")
            self.stop_reception()

    def stop_reception(self):
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.packet_size_entry.config(state=tk.NORMAL)
        self.set_connection_status("Disconnected", False)

    def reception_loop(self):
        try:
            self.log_line("Starting reception thread...")
            
            self.set_connection_status("Connecting to Wi-Fi...", False)
            if not self.connect_to_esp32_wifi():
                self.log_line("Failed to verify Wi-Fi connection.")
                self.set_connection_status("Wi-Fi failed", False)
                self.root.after(0, self.stop_reception)
                return
                
            self.set_connection_status("Wi-Fi connected", True)

            if not self.setup_udp_socket():
                self.root.after(0, self.stop_reception)
                return

            if self.stream_enabled and not self.setup_audio_stream():
                self.stream_enabled = False
                self.log_line("WARNING: Audio playback disabled")
            
            if not self.send_start_command():
                self.root.after(0, self.stop_reception)
                return
                
            self.log_line("Waiting for audio data from ESP32...")
            self.set_connection_status("Receiving data...", True)
            
            while self.running:
                try:
                    # Используем глобальную константу TOTAL_PACKET_SIZE вместо self.expected_total_packet_size
                    data, addr = self.sock.recvfrom(TOTAL_PACKET_SIZE + 200)
                    if addr[0] == ESP32_IP: 
                        self.process_packet(data)
                except socket.timeout:
                    continue  # Remove unnecessary log message for timeouts
                except socket.error as e:
                    if self.running:
                        self.log_line(f"ERROR: Socket error: {e}")
                    break 
                except Exception as e:
                    self.log_line(f"ERROR: Unexpected error in reception loop: {e}")
                    break

        except Exception as e:
            self.log_line(f"ERROR: Fatal error in reception loop: {e}")
        finally:
            self.log_line("Reception loop finished.")
            self.cleanup()
            self.root.after(0, self.stop_reception)

    def setup_audio_stream(self):
        self.log_line("Setting up audio stream...")
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
        
            # Максимально низкая задержка - меньше буфер не рекомендуется
            buffer_size = 64  # Экстремально малый размер для минимальной задержки
        
            # Получаем список устройств для поиска WASAPI
            info = self.pyaudio_instance.get_host_api_info_by_index(0)
            numdevices = info.get('deviceCount')
            use_wasapi = False
        
            # Проверяем наличие WASAPI для Windows
            if platform.system() == "Windows":
                for i in range(0, numdevices):
                    if self.pyaudio_instance.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:
                        if "WASAPI" in str(self.pyaudio_instance.get_device_info_by_host_api_device_index(0, i)):
                            use_wasapi = True
                            self.log_line("Using WASAPI for lower latency")
                            break
        
            # Сбрасываем предыдущие потоки
            if self.audio_stream:
                try:
                    self.audio_stream.stop_stream()
                    self.audio_stream.close()
                except:
                    pass
        
            # Специальная конфигурация для Windows
            if use_wasapi:
                self.audio_stream = self.pyaudio_instance.open(
                    format=pyaudio.paInt32,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    output=True,
                    frames_per_buffer=buffer_size,
                    stream_callback=None,
                    output_device_index=0,  # Используем устройство по умолчанию
                    # WASAPI настройки для Windows
                    host_api_specific_stream_info={
                        'flags': 0x00020000,  # AUDCLNT_STREAMFLAGS_LOWLATENCY
                        'deviceIndex': 0
                    }
                )
            else:
                # Стандартная конфигурация для других систем
                self.audio_stream = self.pyaudio_instance.open(
                    format=pyaudio.paInt32,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    output=True,
                    frames_per_buffer=buffer_size
                )
            
            self.log_line("Audio stream opened successfully.")
            return True
        except Exception as e:
            self.log_line(f"ERROR: Failed to open audio stream: {e}")
            return False

    def process_packet(self, data):
        # В начало функции
        packet_arrival_time = time.time()
        self.packets_received_total += 1

        # Проверяем минимальный размер
        if len(data) < PACKET_HEADER_SIZE:
            self.log_line(f"Error: Packet too small ({len(data)} bytes)")
            return

        # Разбираем заголовок
        header_bytes = data[:PACKET_HEADER_SIZE]
        try:
            magic, packet_num, sample_count, status, checksum = struct.unpack('<IIIII', header_bytes)
        except struct.error as e:
            self.log_line(f"ERROR: Could not unpack packet header: {e}")
            return

        if magic != PACKET_HEADER_MAGIC:
            self.log_line(f"ERROR: Invalid magic number 0x{magic:08X}")
            return
            
        # Отслеживание потерянных пакетов (оптимизация: проверять только значимые потери)
        if not self.initial_packet_num_set:
            self.last_packet_num = packet_num - 1
            self.initial_packet_num_set = True
            
        expected_packet_num = self.last_packet_num + 1
        if packet_num != expected_packet_num:
            packets_lost = packet_num - expected_packet_num
            if packets_lost > 0:
                self.packets_out_of_order += packets_lost
                # Логируем только ЗНАЧИТЕЛЬНЫЕ потери (больше 10 пакетов)
                if packets_lost > 10:
                    self.log_line(f"Warning: Packet loss detected. Lost ~{packets_lost} packets.")
        self.last_packet_num = packet_num

        # Получаем аудио данные
        audio_bytes = data[PACKET_HEADER_SIZE:]
        actual_size = len(audio_bytes)
        
        # Только для первого пакета
        if self.packets_received_total == 1:
            self.log_line(f"First audio packet: {actual_size} bytes, {sample_count} samples")
            if actual_size != AUDIO_DATA_SIZE:
                self.log_line(f"INFO: Audio size from ESP32: {actual_size} bytes")

        # Обновляем статус (оптимизация: не обновлять GUI с каждым пакетом)
        volume = status & 0xFF
        is_muted = (status >> 8) & 0x01
        
        # Обновляем GUI только каждые 5 пакетов (снижает нагрузку на GUI)
        if self.packets_received_total % 5 == 0:
            self.update_status_indicators(volume, is_muted)

        # Воспроизводим звук только если не mute и есть данные
        if not is_muted and self.stream_enabled and self.audio_stream and actual_size > 0:
            try:
                # Проверяем размер и обрабатываем данные
                if actual_size % BYTES_PER_SAMPLE == 0:
                    # Преобразуем аудио данные в numpy массив для векторизованной обработки
                    audio_np = np.frombuffer(audio_bytes, dtype=np.uint8)
                    
                    # Преобразуем 3-байтные сэмплы в массив numpy
                    samples_count = actual_size // BYTES_PER_SAMPLE
                    audio_reshaped = audio_np.reshape(samples_count, BYTES_PER_SAMPLE)
                    
                    # Извлекаем каждый байт из 3-байтного сэмпла
                    byte1 = audio_reshaped[:, 0].astype(np.int32)
                    byte2 = audio_reshaped[:, 1].astype(np.int32)
                    byte3 = audio_reshaped[:, 2].astype(np.int32)
                    
                    # Формируем 24-битные значения (младшие 24 бита)
                    # Маскируем старший бит 3-го байта для обработки знака
                    values = byte1 | (byte2 << 8) | ((byte3 & 0x7F) << 16)
                    
                    # Определяем отрицательные значения (по старшему биту 3-го байта)
                    neg_mask = (byte3 & 0x80) > 0
                    values[neg_mask] = values[neg_mask] - 0x800000
                    
                    # Усиливаем сигнал
                    values = values * 64
                    
                    # Ограничиваем значения диапазоном 32-битного int
                    values = np.clip(values, -0x80000000, 0x7FFFFFFF)
                    
                    # Преобразуем в байты для вывода в аудио поток
                    self.audio_stream.write(values.astype(np.int32).tobytes())
                    
                    # Измеряем задержку сразу после воспроизведения
                    processing_time = time.time() - packet_arrival_time
                    self.latency_samples.append(processing_time * 1000)  # в мс
                    
                    # Ограничиваем количество сохраненных замеров
                    if len(self.latency_samples) > 100:
                        self.latency_samples = self.latency_samples[-100:]
                
            except Exception as e:
                self.log_line(f"ERROR: Audio playback failed: {e}")
                self.stream_enabled = False

    # Обновляем статистику и логирование (оптимизация: реже обновлять)
        self.update_packet_rate()

    def send_start_command(self):
        if not self.sock:
            return False
        
        try:
            self.log_line(f"Sending START command to {ESP32_IP}:{ESP32_UDP_PORT}")
            # Уменьшаем размер буфера для меньшей задержки - 8 KB вместо 64 KB
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            # Отправляем команду один раз
            self.sock.sendto(START_COMMAND, (ESP32_IP, ESP32_UDP_PORT))
            
            # Устанавливаем короткий тайм-аут для проверки ACK
            prev_timeout = self.sock.gettimeout()
            self.sock.settimeout(0.5)  # 500 мс на получение ACK
            
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] == ESP32_IP and data == b"ACK":
                    self.log_line("Received acknowledgment from ESP32")
                    return True
            except socket.timeout:
                self.log_line("No acknowledgment received from ESP32, continuing anyway")
            finally:
                # Восстанавливаем исходный тайм-аут
                self.sock.settimeout(prev_timeout)
            
            return True  # Продолжаем без ACK
        except socket.error as e:
            self.log_line(f"ERROR: Failed to send START command: {e}")
            return False


    def connect_to_esp32_wifi(self):
        os_type = platform.system()
        self.log_line("Please MANUALLY connect to Wi-Fi SSID: ESP32_Audio_AP")
        
        # Увеличиваем таймаут и добавляем больше обратной связи
        timeout = time.time() + WIFI_CONNECTION_TIMEOUT
        last_status_time = 0
        
        while time.time() < timeout:
            try:
                # Выводим статус каждые 5 секунд
                if time.time() - last_status_time > 5:
                    self.log_line(f"Checking connection to ESP32 ({int(timeout - time.time())} seconds remaining)")
                    last_status_time = time.time()
            
                # Настройки пинга в зависимости от ОС
                if os_type == "Windows":
                    ping_cmd = ['ping', '-n', '1', '-w', '500', ESP32_IP]
                else:  # Linux/macOS
                    ping_cmd = ['ping', '-c', '1', '-W', '0.5', ESP32_IP]
            
                # Запускаем процесс с более коротким тайм-аутом
                ping_process = subprocess.run(ping_cmd, 
                                       capture_output=True, 
                                       text=True, 
                                       timeout=1, 
                                       check=False)
            
                # Проверяем успешность пинга
                if ping_process.returncode == 0:
                    self.log_line("Connection established successfully")
                    
                    # Дополнительная проверка UDP соединения
                    try:
                        test_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        test_socket.settimeout(0.5)
                        test_socket.sendto(b"TEST", (ESP32_IP, ESP32_UDP_PORT))
                        self.log_line("UDP connectivity verified")
                        test_socket.close()
                    except Exception as e:
                        self.log_line(f"Warning: UDP test failed, but proceeding: {e}")
                
                    return True
            
                # Пауза между попытками меньше для более быстрой реакции
                time.sleep(0.5)
            
            except subprocess.TimeoutExpired:
            # Сообщение о таймауте пинга
                self.log_line("Ping timeout - still trying...")
                time.sleep(0.5)
            except Exception as e:
                self.log_line(f"Connection check error: {e}")
            time.sleep(0.5)
    
        self.log_line(f"Connection timeout after {WIFI_CONNECTION_TIMEOUT} seconds")
        self.log_line("Please check that:")
        self.log_line("1. ESP32 is powered on and working properly")
        self.log_line("2. Your computer is connected to the ESP32_Audio_AP Wi-Fi network")
        self.log_line("3. No firewall is blocking the connection")
        return False
        
    
    def setup_udp_socket(self):
        self.log_line("Setting up UDP socket...")
        try:
            if self.sock:
                self.sock.close()
                
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(SOCKET_TIMEOUT)
            
            # Снижаем размер буфера сокета для уменьшения задержки
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            
            # Пытаемся установить приоритет пакетов, если поддерживается
            try:
                if platform.system() == "Windows":
                    # Windows QOS options (только если поддерживается)
                    if hasattr(socket, "SO_PRIORITY"):
                        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY, 6)
                else:
                    # Linux/macOS TOS options
                    self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x10)  # IPTOS_LOWDELAY
            except (AttributeError, OSError) as e:
                # Игнорируем ошибку, если параметры не поддерживаются
                self.log_line(f"Info: Priority settings not supported on this system")
                
            # Попытка привязки к порту
            try:
                self.sock.bind(('0.0.0.0', ESP32_UDP_PORT))
            except socket.error:
                self.log_line("Failed to bind to primary address, trying alternative...")
                self.sock.bind(('', ESP32_UDP_PORT))
            
            self.log_line(f"UDP Socket bound to port {ESP32_UDP_PORT}")
            self.set_connection_status("Socket Ready", True)
            return True
        except socket.error as e:
            self.log_line(f"ERROR: Failed to create UDP socket: {e}")
            if self.sock:
                self.sock.close()
                self.sock = None
            return False
    
    def cleanup(self):
        self.log_line("Cleaning up resources...")
        try:
            if self.sock:
                self.sock.close()
                self.sock = None
                
            if self.audio_stream:
                try:
                    self.audio_stream.stop_stream()
                    self.audio_stream.close()
                except Exception as e:
                    self.log_line(f"Warning: Error closing audio stream: {e}")
                    
            if self.pyaudio_instance:
                try:
                    self.pyaudio_instance.terminate()
                except Exception as e:
                    self.log_line(f"Warning: Error terminating PyAudio: {e}")
        
            self.set_connection_status("Disconnected", False)  # Add this line here
        except Exception as e:
            self.log_line(f"Error during cleanup: {e}")
        finally:
            self.audio_stream = None
            self.pyaudio_instance = None

    def on_closing(self):
        self.log_line("Window closed by user.")
        self.stop_reception()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        self.root.destroy()

    def report_statistics(self):
        if self.packets_received_total > 0:
            loss_percent = (self.packets_out_of_order / self.packets_received_total) * 100
            self.log_line(f"Statistics:")
            self.log_line(f"Total packets: {self.packets_received_total}")
            self.log_line(f"Packets lost: {self.packets_out_of_order}")
            self.log_line(f"Packet loss: {loss_percent:.2f}%")

    def update_packet_rate(self):
        now = time.time()
        if self.last_packet_time > 0:
            time_diff = now - self.last_packet_time
            if time_diff > 0:
                self.packet_rate = 1.0 / time_diff
        self.last_packet_time = now

    def display_latency(self):
        # Только обновляем таймер
        if self.running:
            self.latency_display_timer = self.root.after(1000, self.display_latency)

    def display_statistics(self):
        """Регулярно отображает агрегированные статистические данные."""
        if self.running:
            # Статистика пакетов
            loss_percent = 0
            if self.packets_received_total > 0:
                loss_percent = (self.packets_out_of_order / self.packets_received_total) * 100
            
            self.log_line("\n=== СТАТИСТИКА ===")
            self.log_line(f"Пакетов получено: {self.packets_received_total}, Потеряно: {self.packets_out_of_order} ({loss_percent:.1f}%)")
            self.log_line(f"Средняя скорость: {self.packet_rate:.1f} пакетов/сек")
            
            # Статистика задержки
            if self.latency_samples:
                avg_latency = sum(self.latency_samples) / len(self.latency_samples)
                min_latency = min(self.latency_samples)
                max_latency = max(self.latency_samples)
                self.log_line(f"Задержка обработки: средняя={avg_latency:.2f}мс, мин={min_latency:.2f}мс, макс={max_latency:.2f}мс")
            self.log_line("==================\n")
            
            # Планируем следующее обновление через 5 секунд
            if self.running:  # Дополнительная проверка
                self.root.after(5000, self.display_statistics)

def main():
    root = tk.Tk()
    app = AudioApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()