#!/usr/bin/env python3
"""
Text-to-speech functionality using OpenAI's TTS API.
Requires OPENAI_API_KEY environment variable to be set.
"""

import os
import io
import wave
import pyaudio
import tempfile
import threading
import numpy as np
from scipy.io import wavfile
from scipy import signal
from openai import OpenAI

# Completely suppress ALSA error messages
import ctypes
from contextlib import contextmanager

@contextmanager
def suppress_alsa_errors():
    # Save the original error handler
    libc = ctypes.CDLL('libc.so.6')
    stderr_fd = 2  # stderr file descriptor
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(stderr_fd)
    try:
        # Redirect stderr to /dev/null
        os.dup2(devnull, stderr_fd)
        yield
    finally:
        # Restore stderr
        os.dup2(old_stderr, stderr_fd)
        os.close(old_stderr)
        os.close(devnull)

# Configuration
OPENAI_TTS_MODEL = "tts-1"  # or "tts-1-hd" for higher quality
OPENAI_TTS_VOICE = "ash"  # alloy, echo, fable, onyx, nova, or shimmer

# Audio settings
CHUNK = 1024

def list_audio_devices():
    """List all available audio devices"""
    p = pyaudio.PyAudio()
    print("\n=== Available Audio Devices ===")
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        print(f"Device {i}: {dev['name']}")
        print(f"   Input Channels: {dev['maxInputChannels']}, Output Channels: {dev['maxOutputChannels']}")
        print(f"   Default Sample Rate: {dev['defaultSampleRate']} Hz")
        print(f"   Default Output: {'Yes' if dev.get('isDefaultOutput', False) else 'No'}")
    print("==============================\n")
    p.terminate()

def speak(text: str, voice: str = None, model: str = None):
    """
    Convert text to speech and play it using the ReSpeaker 4 Mic Array audio output.
    
    Args:
        text: The text to speak
        voice: Optional voice to use (overrides default)
        model: Optional model to use (overrides default)
        
    Returns:
        bool: True if successful, False otherwise
    """
    print(f"Speaking: {text}")
    
    # Initialize OpenAI client
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY not set")
        return False

    client = OpenAI(api_key=api_key)
    voice = voice or OPENAI_TTS_VOICE
    model = model or OPENAI_TTS_MODEL
    
    # Audio device configuration
    RESPEAKER_DEVICE_INDEX = 0  # ReSpeaker 4 Mic Array
    INPUT_SAMPLE_RATE = 24000  # OpenAI's default output sample rate
    OUTPUT_SAMPLE_RATE = 16000  # ReSpeaker's supported sample rate
    
    # Suppress ALSA errors during audio playback
    with suppress_alsa_errors():
        try:
            # Request audio from OpenAI TTS
            print("Requesting TTS from OpenAI...")
            response = client.audio.speech.create(
                model=model,
                voice=voice,
                input=text,
                response_format="wav",
                speed=1.2  # Slightly faster for better clarity
            )
            print("Received TTS response from OpenAI")
            
            # Save to a temporary file
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
                audio_data = b''
                for chunk in response.iter_bytes():
                    if chunk:
                        audio_data += chunk
                tmp_file.write(audio_data)
                tmp_file_path = tmp_file.name
            
            try:
                # Open the WAV file
                wf = wave.open(tmp_file_path, 'rb')
                
                # Initialize PyAudio
                print("Initializing PyAudio...")
                p = pyaudio.PyAudio()
                
                try:
                    # Read the WAV file
                    wf.rewind()
                    audio_data = wf.readframes(wf.getnframes())
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)
                    
                    # Resample from 24kHz to 16kHz if needed
                    if wf.getframerate() != OUTPUT_SAMPLE_RATE:
                        num_samples = int(len(audio_array) * OUTPUT_SAMPLE_RATE / wf.getframerate())
                        audio_array = signal.resample(audio_array, num_samples).astype(np.int16)
                    
                    # Open audio stream with device's sample rate
                    print(f"Opening audio stream at {OUTPUT_SAMPLE_RATE}Hz...")
                    stream = p.open(
                        format=p.get_format_from_width(wf.getsampwidth()),
                        channels=min(2, wf.getnchannels()),  # Force max 2 channels
                        rate=OUTPUT_SAMPLE_RATE,
                        output=True,
                        output_device_index=RESPEAKER_DEVICE_INDEX,
                        start=False,
                        frames_per_buffer=CHUNK
                    )
                    print(f"Audio stream opened with format: {stream._format}, channels: {stream._channels}, rate: {stream._rate}")
                    
                except Exception as e:
                    p.terminate()
                    raise
                
                try:
                    # Start the stream and play the audio
                    print("Starting audio playback...")
                    stream.start_stream()
                    
                    # Write audio in chunks with error handling
                    chunk_size = CHUNK * 2  # For 16-bit audio
                    total_bytes = 0
                    for i in range(0, len(audio_array), chunk_size):
                        chunk = audio_array[i:i + chunk_size].tobytes()
                        stream.write(chunk)
                        total_bytes += len(chunk)
                    
                    print(f"Sent {total_bytes} bytes to audio device")
                    
                    # Wait for the stream to finish playing
                    while stream.is_active():
                        import time
                        time.sleep(0.1)
                    
                    print("Playback complete")
                finally:
                    # Clean up resources
                    stream.stop_stream()
                    stream.close()
                    p.terminate()
                
                # Close the WAV file
                wf.close()
                return True
                
            except Exception as e:
                return False
                
            finally:
                # Clean up the temporary file
                try:
                    os.unlink(tmp_file_path)
                except Exception:
                    pass
                    
        except Exception as e:
            print(f"Error in speak(): {e}")
            import traceback
            traceback.print_exc()
            return False

def speak_in_thread(text: str):
    """Run speak in a non-daemon thread to ensure completion."""
    def _speak():
        try:
            speak(text)
        except Exception as e:
            print(f"Error in speak thread: {e}")
    
    thread = threading.Thread(target=_speak)
    thread.daemon = False  # Ensure thread completes even if main thread exits
    thread.start()
    return thread

def test():
    """Test the TTS functionality with a sample phrase."""
    print("Testing text-to-speech...")
    success = speak("Hello! This is a test of the text-to-speech system.")
    if success:
        print("Test completed successfully!")
    else:
        print("Test failed.")

if __name__ == "__main__":
    test()
