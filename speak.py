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
    # Initialize OpenAI client
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set")
        return False

    print(f"Speaking: {text}")
    
    client = OpenAI(api_key=api_key)
    voice = voice or OPENAI_TTS_VOICE
    model = model or OPENAI_TTS_MODEL
    
    # Audio device configuration
    RESPEAKER_DEVICE_INDEX = 0  # ReSpeaker 4 Mic Array
    INPUT_SAMPLE_RATE = 24000  # OpenAI's default output sample rate
    OUTPUT_SAMPLE_RATE = 16000  # ReSpeaker's supported sample rate
    
    # List available audio devices for debugging
    list_audio_devices()
    print(f"\n=== Using Device {RESPEAKER_DEVICE_INDEX} (ReSpeaker 4 Mic Array) at {OUTPUT_SAMPLE_RATE}Hz ===\n")
    
    # Suppress ALSA errors during audio playback
    with suppress_alsa_errors():
        try:
            # Request audio from OpenAI TTS
            print(f"Requesting TTS (will resample to {OUTPUT_SAMPLE_RATE}Hz for output)")
            response = client.audio.speech.create(
                model=model,
                voice=voice,
                input=text,
                response_format="wav",
                speed=1.3  # Slightly faster for better clarity
            )
            print("Received audio response from OpenAI")
            
            # Save to a temporary file
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
                audio_data = b''
                for chunk in response.iter_bytes():
                    if chunk:
                        audio_data += chunk
                tmp_file.write(audio_data)
                tmp_file_path = tmp_file.name
                print(f"Saved audio to temporary file: {tmp_file_path}")
                print(f"Audio data size: {len(audio_data)} bytes")
            
            try:
                # Open the WAV file
                print("\n=== Analyzing WAV file...")
                wf = wave.open(tmp_file_path, 'rb')
                print(f"WAV file info: {wf.getnchannels()} channels, {wf.getsampwidth()} bytes/sample, {wf.getframerate()} Hz")
                print(f"WAV file frames: {wf.getnframes()}, duration: {wf.getnframes()/wf.getframerate():.2f} seconds")
                
                # Initialize PyAudio with error handling
                print("\n=== Initializing PyAudio...")
                p = pyaudio.PyAudio()
                
                # Get ReSpeaker device info
                try:
                    device_info = p.get_device_info_by_index(RESPEAKER_DEVICE_INDEX)
                    print(f"Using output device {RESPEAKER_DEVICE_INDEX}: {device_info['name']}")
                    print(f"  Max output channels: {device_info['maxOutputChannels']}")
                    print(f"  Default sample rate: {device_info['defaultSampleRate']} Hz")
                    
                    # Get the actual sample rate from the WAV file
                    sample_rate = int(wf.getframerate())
                    print(f"WAV file sample rate: {sample_rate} Hz")
                    
                    # Read the WAV file
                    wf.rewind()
                    audio_data = wf.readframes(wf.getnframes())
                    audio_array = np.frombuffer(audio_data, dtype=np.int16)
                    
                    # Resample from 24kHz to 16kHz if needed
                    if wf.getframerate() != OUTPUT_SAMPLE_RATE:
                        print(f"Resampling from {wf.getframerate()}Hz to {OUTPUT_SAMPLE_RATE}Hz")
                        num_samples = int(len(audio_array) * OUTPUT_SAMPLE_RATE / wf.getframerate())
                        audio_array = signal.resample(audio_array, num_samples).astype(np.int16)
                    
                    # Open audio stream with device's sample rate
                    print("\n=== Opening audio stream...")
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
                    print(f"Error initializing audio device: {str(e)}")
                    raise
                print(f"Audio stream opened with format: {stream._format}, channels: {stream._channels}, rate: {stream._rate}")
                
                # Start the stream
                print("Starting audio playback...")
                stream.start_stream()
                
                # Play the resampled audio
                print("Starting audio playback...")
                stream.start_stream()
                
                # Write audio in chunks
                chunk_size = CHUNK * 2  # For 16-bit audio
                total_bytes = 0
                for i in range(0, len(audio_array), chunk_size):
                    chunk = audio_array[i:i + chunk_size].tobytes()
                    stream.write(chunk)
                    total_bytes += len(chunk)
                
                print(f"Playback complete. Total bytes played: {total_bytes}")
                
                # Cleanup
                stream.stop_stream()
                stream.close()
                p.terminate()
                wf.close()
                return True
                
            except Exception as e:
                print(f"Error during audio playback: {str(e)}")
                import traceback
                traceback.print_exc()
                return False
                
            finally:
                # Clean up the temporary file
                try:
                    os.unlink(tmp_file_path)
                    print(f"Deleted temporary file: {tmp_file_path}")
                except Exception as e:
                    print(f"Error deleting temporary file: {e}")
                    
        except Exception as e:
            print(f"Error in speak(): {e}")
            import traceback
            traceback.print_exc()
            return False

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
