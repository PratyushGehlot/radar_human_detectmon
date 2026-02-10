"""
Convert MP3/WAV audio files to the WAV format required by ESP32-S3 Box 3.
Target: 22050 Hz, 16-bit, mono, PCM WAV

Requirements: pip install pydub
Also requires ffmpeg installed on your system (for MP3 support).
"""

import sys
import os
import wave
import struct
from pathlib import Path

try:
    from pydub import AudioSegment
except ImportError:
    print("ERROR: pydub is required. Install with: pip install pydub")
    print("       For MP3 support, also install ffmpeg.")
    sys.exit(1)

TARGET_SAMPLE_RATE = 22050
TARGET_CHANNELS = 1
TARGET_SAMPLE_WIDTH = 2  # 16-bit = 2 bytes


def inspect_wav(filepath):
    """Print WAV file properties."""
    try:
        with wave.open(str(filepath), "rb") as wf:
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            framerate = wf.getframerate()
            frames = wf.getnframes()
            duration = frames / framerate if framerate > 0 else 0

            print(f"  Channels:        {channels} ({'mono' if channels == 1 else 'stereo'})")
            print(f"  Sample rate:     {framerate} Hz")
            print(f"  Bits per sample: {sample_width * 8}")
            print(f"  Duration:        {duration:.2f}s")
            print(f"  Frames:          {frames}")

            ok = (
                channels == TARGET_CHANNELS
                and sample_width == TARGET_SAMPLE_WIDTH
                and framerate == TARGET_SAMPLE_RATE
            )
            if ok:
                print("  Status:          COMPATIBLE with ESP32-S3 Box 3")
            else:
                print("  Status:          NEEDS CONVERSION")
                if channels != TARGET_CHANNELS:
                    print(f"                   - channels: {channels} -> {TARGET_CHANNELS}")
                if framerate != TARGET_SAMPLE_RATE:
                    print(f"                   - sample rate: {framerate} -> {TARGET_SAMPLE_RATE}")
                if sample_width != TARGET_SAMPLE_WIDTH:
                    print(f"                   - bit depth: {sample_width*8} -> {TARGET_SAMPLE_WIDTH*8}")
            return ok
    except wave.Error as e:
        print(f"  Not a valid WAV file: {e}")
        return False


def convert_audio(input_path, output_path=None):
    """Convert any audio file to ESP32-S3 Box 3 compatible WAV."""
    input_path = Path(input_path)
    if not input_path.exists():
        print(f"ERROR: File not found: {input_path}")
        return False

    ext = input_path.suffix.lower()
    if output_path is None:
        stem = input_path.stem
        output_path = input_path.parent / f"{stem}_esp32.wav"
    output_path = Path(output_path)

    print(f"\nInput:  {input_path}")

    if ext == ".wav":
        print("Format: WAV")
        inspect_wav(input_path)
    elif ext == ".mp3":
        print("Format: MP3")
    else:
        print(f"Format: {ext}")

    print(f"\nConverting to ESP32-S3 Box 3 format...")
    print(f"  Target: {TARGET_SAMPLE_RATE} Hz, {TARGET_SAMPLE_WIDTH*8}-bit, mono, PCM")

    try:
        if ext == ".mp3":
            audio = AudioSegment.from_mp3(str(input_path))
        elif ext == ".wav":
            audio = AudioSegment.from_wav(str(input_path))
        elif ext == ".ogg":
            audio = AudioSegment.from_ogg(str(input_path))
        elif ext == ".flac":
            audio = AudioSegment.from_file(str(input_path), format="flac")
        else:
            audio = AudioSegment.from_file(str(input_path))

        audio = audio.set_channels(TARGET_CHANNELS)
        audio = audio.set_frame_rate(TARGET_SAMPLE_RATE)
        audio = audio.set_sample_width(TARGET_SAMPLE_WIDTH)

        audio.export(str(output_path), format="wav")

        print(f"\nOutput: {output_path}")
        inspect_wav(output_path)
        print(f"\nFile size: {output_path.stat().st_size} bytes")
        return True

    except Exception as e:
        print(f"ERROR: Conversion failed: {e}")
        return False


def main():
    if len(sys.argv) < 2:
        print("ESP32-S3 Box 3 Audio Converter")
        print(f"Target format: {TARGET_SAMPLE_RATE} Hz, {TARGET_SAMPLE_WIDTH*8}-bit, mono, PCM WAV")
        print()
        print("Usage:")
        print(f"  python {sys.argv[0]} <input_file> [output_file]")
        print(f"  python {sys.argv[0]} --check <wav_file>")
        print()
        print("Examples:")
        print(f"  python {sys.argv[0]} alert.mp3                  # -> alert_esp32.wav")
        print(f"  python {sys.argv[0]} alert.mp3 output.wav       # -> output.wav")
        print(f"  python {sys.argv[0]} --check bootaudio.wav      # inspect only")
        sys.exit(1)

    if sys.argv[1] == "--check":
        if len(sys.argv) < 3:
            print("ERROR: Provide a WAV file to check.")
            sys.exit(1)
        filepath = Path(sys.argv[2])
        if not filepath.exists():
            print(f"ERROR: File not found: {filepath}")
            sys.exit(1)
        print(f"Checking: {filepath}")
        ok = inspect_wav(filepath)
        sys.exit(0 if ok else 1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    success = convert_audio(input_file, output_file)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
