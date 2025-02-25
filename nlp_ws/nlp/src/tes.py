# import os
# import json
# import queue
# import subprocess
# import pyaudio
# from vosk import Model, KaldiRecognizer

# # Define wake word
# WAKE = "hello nico"

# # Load Vosk model
# MODEL_PATH = "vosk_models/vosk-model-small-en-us-0.15"
# model = Model(MODEL_PATH)

# # Audio streaming parameters
# SAMPLE_RATE = 16000
# CHUNK_SIZE = 4000  # Adjust buffer size if necessary

# # Initialize PyAudio
# p = pyaudio.PyAudio()

# def get_microphone_index():
#     """Finds the microphone index for the specified device."""
#     for index in range(p.get_device_count()):
#         info = p.get_device_info_by_index(index)
#         if "AB13X" in info["name"]:  # Adjust for your specific microphone name
#             print(f"Using microphone: {info['name']}")
#             return index
#     return None  # If no matching microphone is found

# device_index = get_microphone_index()
# if device_index is None:
#     raise ValueError("Microphone AB13X not found! Check your input device.")

# def listen_and_recognize():
#     """Recognizes voice commands from the microphone in real-time."""
#     stream = p.open(format=pyaudio.paInt16,
#                     channels=1,
#                     rate=SAMPLE_RATE,
#                     input=True,
#                     frames_per_buffer=CHUNK_SIZE,
#                     input_device_index=device_index)

#     recognizer = KaldiRecognizer(model, SAMPLE_RATE)
#     recognizer.SetWords(True)  # Enable word-by-word output

#     print("\n🎤 Ready. Say 'Hello Nico' to activate.")

#     while True:
#         try:
#             data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
            
#             if recognizer.AcceptWaveform(data):
#                 result = json.loads(recognizer.Result())
#                 transcript = result.get("text", "").lower()
                
#                 if transcript:
#                     print(f"🎤 Heard: {transcript}")

#                     if WAKE in transcript:
#                         print("🔊 Wake word detected! Listening for command...")
#                         speak_text("Yes, I am listening.")

#                         # Listen for command
#                         data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
#                         if recognizer.AcceptWaveform(data):
#                             result = json.loads(recognizer.Result())
#                             command_text = result.get("text", "").lower()

#                             if not command_text:
#                                 print("⚠ No command detected.")
#                                 continue

#                             if "stop" in command_text:
#                                 print("🛑 Stop command received.")
#                                 speak_text("Stopping.")
#                                 continue

#                             # Process the command
#                             response = process_command(command_text)
#                             print(f"📢 Command: [{response}] {command_text}")
#                             speak_text(response)
                            
#         except KeyboardInterrupt:
#             print("\n🔴 Stopping...")
#             stream.stop_stream()
#             stream.close()
#             p.terminate()
#             break
#         except Exception as e:
#             print(f"Error: {e}")

# def process_command(command):
#     """Processes recognized speech and returns a response."""
#     if "weather" in command:
#         return "The weather is sunny and 25 degrees."
#     elif "time" in command:
#         from datetime import datetime
#         return f"The current time is {datetime.now().strftime('%H:%M')}."
#     elif "joke" in command:
#         return "Why don't robots ever get tired? Because they recharge!"
#     else:
#         return "I'm sorry, I didn't understand that."

# def speak_text(text, language="en-US", output_file="output.wav"):
#     """Converts text to speech using pico2wave and plays the output."""
#     try:
#         subprocess.run(["pico2wave", "-l", language, "-w", output_file, text], check=True)
#         print(f"🗣️ Saying: {text}")
#         play_audio(output_file)
#     except subprocess.CalledProcessError as e:
#         print(f"Error: {e}")

# def play_audio(file_path, delete_after=True):
#     """Plays an audio file."""
#     try:
#         subprocess.run(["aplay", file_path], check=True)
#     except subprocess.CalledProcessError as e:
#         print(f"Error: {e}")

#     if delete_after:
#         os.remove(file_path)

# if __name__ == "__main__":
#     listen_and_recognize()


import pyaudio
p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    d = p.get_device_info_by_index(i)
    print(f"{i}:{d['name']}, {d['defaultSampleRate']}")