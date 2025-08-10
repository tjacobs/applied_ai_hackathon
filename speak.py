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

def speak(text: str, voice: str = None, model: str = None):
    """
    Convert text to speech and play it using the default audio output.
    
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
        
    client = OpenAI(api_key=api_key)
    voice = voice or OPENAI_TTS_VOICE
    model = model or OPENAI_TTS_MODEL
    
    # Suppress ALSA errors during audio playback
    with suppress_alsa_errors():
        try:
            # Request audio from OpenAI TTS (request WAV format for direct playback)
            response = client.audio.speech.create(
                model=model,
                voice=voice,
                input=text,
                response_format="wav",
            )
            
            # Save to a temporary file
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp_file:
                for chunk in response.iter_bytes():
                    if chunk:
                        tmp_file.write(chunk)
                tmp_file_path = tmp_file.name
            
            try:
                # Open the WAV file
                wf = wave.open(tmp_file_path, 'rb')
                
                # Initialize PyAudio with error handling
                p = pyaudio.PyAudio()
                
                # Get default output device info
                default_output = p.get_default_output_device_info()
                
                # Open a stream with explicit device and error handling
                stream = p.open(
                    format=p.get_format_from_width(wf.getsampwidth()),
                    channels=min(2, wf.getnchannels()),  # Force max 2 channels
                    rate=wf.getframerate(),
                    output=True,
                    output_device_index=default_output['index'],
                    start=False
                )
                
                # Start the stream
                stream.start_stream()
                
                # Read data in chunks and play
                data = wf.readframes(CHUNK)
                while data:
                    stream.write(data)
                    data = wf.readframes(CHUNK)
                
                # Cleanup
                stream.stop_stream()
                stream.close()
                p.terminate()
                wf.close()
                return True
                
            finally:
                # Clean up the temporary file
                try:
                    os.unlink(tmp_file_path)
                except:
                    pass
                    
        except Exception as e:
            print(f"Error in speak(): {e}")
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
